"""ROS/PGM orientation, durable revisions, edits, import and route contracts."""

import io
import json
import zipfile

from PIL import Image
import pytest
import yaml

from kiwi_web.maps import MapStore, pose


@pytest.fixture
def store(tmp_path):
    storage = MapStore(tmp_path)
    storage.save_grid(
        "lab",
        dict(width=3, height=2, resolution=0.05, origin=[-2, 3, 0.7]),
        [0, 100, -1, 100, 0, -1],
    )
    return storage


def test_grid_orientation_and_png(store):
    document = store.read("lab")
    assert list(document["image"].getdata()) == [0, 254, 205, 254, 0, 205]
    assert document["origin"] == [-2, 3, 0.7]
    png, revision = store.png("lab")
    assert list(Image.open(io.BytesIO(png)).getdata()) == [0, 254, 205, 254, 0, 205]
    assert revision == document["revision"]


def test_atomic_edit_rejects_stale_revision_and_invalid_batch(store):
    original = store.read("lab")
    with pytest.raises(ValueError):
        store.edit("lab", original["revision"], [dict(value=0, indices=[0, 1, 99])])
    assert store.read("lab")["revision"] == original["revision"]
    saved = store.edit("lab", original["revision"], [dict(value=0, indices=[1, 3])])
    assert saved["revision"] != original["revision"]
    assert list(store.read("lab")["image"].getdata()) == [0, 0, 205, 0, 0, 205]
    assert len(list(store.versions.glob("*.pgm"))) == 2
    with pytest.raises(ValueError):
        store.edit("lab", original["revision"], [dict(value=254, indices=[0])])


def test_name_and_image_path_are_confined(store, tmp_path):
    for name in ("../escape", "/tmp/lab", "a.b", "", "a" * 65):
        with pytest.raises(ValueError):
            store.read(name)
    (tmp_path / "unsafe.yaml").write_text("image: /etc/passwd\n")
    with pytest.raises(ValueError):
        store.read("unsafe")
    (tmp_path / "linked.yaml").symlink_to(tmp_path / "lab.yaml")
    with pytest.raises(ValueError):
        store.read("linked")


def test_export_import_preserves_pixels_metadata_and_routes(store):
    store.save_route("lab", "round", [dict(x=-1, y=4, yaw=0.5)])
    data = store.export("lab")
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        store.import_map("copy", archive.read("lab.yaml"), archive.read("lab.pgm"))
        assert json.loads(archive.read("lab.routes.json"))["round"][0]["yaw"] == 0.5
    assert store.read("copy")["image"].tobytes() == store.read("lab")["image"].tobytes()
    assert store.read("copy")["origin"] == store.read("lab")["origin"]
    with pytest.raises(ValueError):
        store.save_grid("lab", {}, [])


def test_custom_negate_threshold_edit_keeps_brush_semantics(store):
    image = Image.new("L", (3, 1))
    image.putdata([255, 0, 127])
    buffer = io.BytesIO()
    image.save(buffer, "PPM")
    metadata = dict(
        image="anything.pgm",
        resolution=0.1,
        origin=[0, 0, 1],
        negate=1,
        occupied_thresh=0.7,
        free_thresh=0.3,
    )
    document = store.import_map("negated", yaml.safe_dump(metadata).encode(), buffer.getvalue())
    store.edit("negated", document["revision"], [dict(value=254, indices=[0])])
    updated = store.read("negated")
    assert list(updated["image"].getdata()) == [254, 254, 205]
    assert updated["metadata"]["negate"] == 0


def test_invalid_routes_and_soft_delete(store):
    with pytest.raises(ValueError):
        store.save_route("lab", "bad", [])
    with pytest.raises(ValueError):
        pose(dict(x=float("nan"), y=0, yaw=0))
    store.delete("lab")
    assert store.list() == []
    assert list(store.versions.glob("*deleted*.yaml"))


def test_transparent_png_is_unknown_and_legacy_metadata_is_backed_up(store):
    image = Image.new("RGBA", (2, 1), (255, 255, 255, 255))
    image.putpixel((0, 0), (255, 255, 255, 0))
    buffer = io.BytesIO()
    image.save(buffer, "PNG")
    metadata = dict(
        resolution=0.1, origin=[0, 0, 0], negate=0, occupied_thresh=0.65, free_thresh=0.196
    )
    original = store.import_map("transparent", yaml.safe_dump(metadata).encode(), buffer.getvalue())
    png, _ = store.png("transparent")
    assert list(Image.open(io.BytesIO(png)).getdata()) == [205, 254]
    store.edit("transparent", original["revision"], [dict(value=0, indices=[1])])
    assert list(store.versions.glob("transparent_*_previous.yaml"))


def test_interrupted_commit_keeps_the_previous_map_readable(store, monkeypatch):
    from kiwi_web import maps

    original = store.read("lab")
    write = maps.atomic_write

    def fail_commit(path, data):
        if path == store.path("lab"):
            raise OSError("simulated interrupted write")
        return write(path, data)

    monkeypatch.setattr(maps, "atomic_write", fail_commit)
    with pytest.raises(OSError):
        store.edit("lab", original["revision"], [dict(value=254, indices=[0])])
    assert store.read("lab")["revision"] == original["revision"]
