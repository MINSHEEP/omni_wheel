"""Bounded map storage, optimistic edit revisions and atomic YAML commits."""

import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import threading
import uuid
import zipfile

from PIL import Image, ImageOps
import yaml

MAX_CELLS = 16_000_000


def valid_name(name):
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", name):
        raise ValueError("名称须为 1–64 位英文、数字、下划线或连字符")
    return name


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError("参数必须是有限数值")
    return float(value)


def pose(value):
    if not isinstance(value, dict):
        raise ValueError("位姿格式无效")
    result = {key: finite(value[key]) for key in ("x", "y", "yaw")}
    if abs(result["x"]) > 100000 or abs(result["y"]) > 100000:
        raise ValueError("位姿超出范围")
    result["yaw"] = math.remainder(result["yaw"], math.tau)
    return result


def atomic_write(path, data):
    temporary = path.with_name("." + path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        descriptor = os.open(path.parent, os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        temporary.unlink(missing_ok=True)


def grayscale(source, metadata):
    """Preserve transparent PNG cells as unknown when converting to PGM."""
    image = source.convert("L")
    if source.mode in ("RGBA", "LA") or "transparency" in source.info:
        alpha = source.convert("RGBA").getchannel("A")
        free = finite(metadata.get("free_thresh", 0.196))
        occupied = finite(metadata.get("occupied_thresh", 0.65))
        probability = (free + occupied) / 2
        unknown = round(255 * (probability if metadata.get("negate", 0) else 1 - probability))
        image.paste(unknown, mask=alpha.point(lambda value: 255 if value < 255 else 0))
    return image


class MapStore:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.versions = self.root / ".versions"
        self.versions.mkdir(exist_ok=True)
        self.routes = self.root / ".routes"
        self.routes.mkdir(exist_ok=True)
        self.lock = threading.RLock()

    def path(self, name):
        path = self.root / (valid_name(name) + ".yaml")
        if path.is_symlink():
            raise ValueError("不支持符号链接地图")
        return path

    def list(self):
        result = []
        for path in sorted(self.root.glob("*.yaml")):
            try:
                valid_name(path.stem)
                if not path.is_symlink():
                    result.append(dict(name=path.stem, modified=path.stat().st_mtime))
            except ValueError:
                continue
        return result

    def read(self, name):
        with self.lock:
            raw = self.path(name).read_bytes()
            if len(raw) > 65536:
                raise ValueError("地图 YAML 过大")
            metadata = yaml.safe_load(raw)
            if not isinstance(metadata, dict):
                raise ValueError("地图 YAML 无效")
            image_path = (self.root / str(metadata.get("image", ""))).resolve()
            if not image_path.is_relative_to(self.root) or not image_path.is_file():
                raise ValueError("地图图像必须位于地图目录内")
            if image_path.stat().st_size > MAX_CELLS * 3:
                raise ValueError("地图图像过大")
            image_bytes = image_path.read_bytes()
            with Image.open(io.BytesIO(image_bytes)) as source:
                if source.width * source.height > MAX_CELLS:
                    raise ValueError("地图超过 1600 万栅格限制")
                image = grayscale(source, metadata)
            resolution = finite(metadata.get("resolution"))
            origin = metadata.get("origin")
            if resolution <= 0 or not isinstance(origin, list) or len(origin) != 3:
                raise ValueError("地图分辨率或原点无效")
            origin = [finite(v) for v in origin]
            if metadata.get("mode", "trinary") != "trinary":
                raise ValueError("当前编辑器支持 trinary 栅格地图")
            free = finite(metadata.get("free_thresh", 0.196))
            occupied = finite(metadata.get("occupied_thresh", 0.65))
            if not 0 <= free < occupied <= 1 or metadata.get("negate", 0) not in (0, 1):
                raise ValueError("地图阈值无效")
            revision = hashlib.sha256(raw + image_bytes).hexdigest()
            return dict(
                name=name,
                width=image.width,
                height=image.height,
                resolution=resolution,
                origin=origin,
                revision=revision,
                metadata=metadata,
                image=image,
            )

    @staticmethod
    def public(document):
        return {k: v for k, v in document.items() if k not in ("metadata", "image")}

    @staticmethod
    def occupancy_image(document):
        meta = document["metadata"]
        free, occupied = meta.get("free_thresh", 0.196), meta.get("occupied_thresh", 0.65)
        negate = meta.get("negate", 0)
        table = []
        for value in range(256):
            probability = value / 255 if negate else (255 - value) / 255
            table.append(0 if probability > occupied else 254 if probability < free else 205)
        return document["image"].point(table)

    def png(self, name):
        document = self.read(name)
        out = io.BytesIO()
        self.occupancy_image(document).save(out, "PNG")
        return out.getvalue(), document["revision"]

    def _commit(self, name, image, metadata):
        identifier = name + "_" + uuid.uuid4().hex
        image_path = self.versions / (identifier + ".pgm")
        output = io.BytesIO()
        image.save(output, "PPM")  # L images are written as binary PGM (P5).
        atomic_write(image_path, output.getvalue())
        updated = dict(metadata, image=str(image_path.relative_to(self.root)))
        raw = yaml.safe_dump(updated, allow_unicode=True, sort_keys=False).encode()
        # Keep immutable revisions; YAML is the single transaction commit point.
        atomic_write(self.versions / (identifier + ".yaml"), raw)
        if self.path(name).exists():
            # Also retain the original metadata of maps created outside this
            # package. Their source image is never overwritten by an edit.
            atomic_write(
                self.versions / (identifier + "_previous.yaml"), self.path(name).read_bytes()
            )
        atomic_write(self.path(name), raw)
        return self.public(self.read(name))

    def save_grid(self, name, info, data):
        with self.lock:
            valid_name(name)
            if self.path(name).exists():
                raise ValueError("地图名称已存在，请使用新名称保存")
            width, height = int(info["width"]), int(info["height"])
            if not 0 < width * height <= MAX_CELLS or len(data) != width * height:
                raise ValueError("地图尺寸无效")
            # ROS uses a bottom-up grid; PGM uses top-down raster rows.
            pixels = bytes(
                205 if v < 0 else 0 if v >= 65 else 254 if v <= 25 else 205 for v in data
            )
            image = ImageOps.flip(Image.frombytes("L", (width, height), pixels))
            return self._commit(
                name,
                image,
                dict(
                    resolution=info["resolution"],
                    origin=info["origin"],
                    negate=0,
                    mode="trinary",
                    occupied_thresh=0.65,
                    free_thresh=0.196,
                ),
            )

    def edit(self, name, revision, operations):
        with self.lock:
            document = self.read(name)
            if revision != document["revision"]:
                raise ValueError("地图已被其他页面修改，请重新打开后编辑")
            if not isinstance(operations, list) or len(operations) > 5000:
                raise ValueError("编辑次数过多，请分批保存")
            image = document["image"].copy()
            pixels = bytearray(image.tobytes())
            total = 0
            # Client indices refer to PGM rows, not ROS rows.
            for operation in operations:
                value = operation.get("value")
                indices = operation.get("indices")
                if value not in (0, 205, 254) or not isinstance(indices, list):
                    raise ValueError("编辑内容无效")
                total += len(indices)
                if total > 2_000_000:
                    raise ValueError("本次编辑超过 200 万栅格，请分批保存")
                for index in indices:
                    if type(index) is not int or not 0 <= index < len(pixels):
                        raise ValueError("栅格索引越界")
            # Canonicalize to standard trinary values so custom thresholds/negate
            # cannot reverse the meaning of the user's brush.
            pixels = bytearray(self.occupancy_image(document).tobytes())
            for operation in operations:
                for index in operation["indices"]:
                    pixels[index] = operation["value"]
            image = Image.frombytes("L", image.size, bytes(pixels))
            return self._commit(
                name,
                image,
                dict(document["metadata"], negate=0, free_thresh=0.196, occupied_thresh=0.65),
            )

    def delete(self, name):
        with self.lock:
            document = self.read(name)
            archive = self.versions / (name + "_deleted_" + uuid.uuid4().hex + ".yaml")
            os.replace(self.path(name), archive)
            route_path = self.routes / (name + ".json")
            if route_path.exists():
                os.replace(route_path, archive.with_suffix(".routes.json"))
            return self.public(document)

    def export(self, name):
        document = self.read(name)
        buffer = io.BytesIO()
        pgm = io.BytesIO()
        document["image"].save(pgm, "PPM")
        with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.writestr(name + ".pgm", pgm.getvalue())
            archive.writestr(
                name + ".yaml", yaml.safe_dump(dict(document["metadata"], image=name + ".pgm"))
            )
            archive.writestr(name + ".routes.json", json.dumps(self.list_routes(name)))
        return buffer.getvalue()

    def import_map(self, name, yaml_bytes, image_bytes):
        with self.lock:
            valid_name(name)
            if self.path(name).exists():
                raise ValueError("地图名称已存在")
            if len(yaml_bytes) > 65536 or len(image_bytes) > MAX_CELLS * 3:
                raise ValueError("导入文件过大")
            metadata = yaml.safe_load(yaml_bytes)
            if not isinstance(metadata, dict):
                raise ValueError("地图 YAML 无效")
            resolution = finite(metadata.get("resolution"))
            origin = metadata.get("origin")
            if resolution <= 0 or not isinstance(origin, list) or len(origin) != 3:
                raise ValueError("地图元数据无效")
            origin = [finite(v) for v in origin]
            if metadata.get("mode", "trinary") != "trinary":
                raise ValueError("仅支持 trinary 地图")
            free, occupied = finite(metadata.get("free_thresh", 0.196)), finite(
                metadata.get("occupied_thresh", 0.65)
            )
            if not 0 <= free < occupied <= 1 or metadata.get("negate", 0) not in (0, 1):
                raise ValueError("地图阈值无效")
            with Image.open(io.BytesIO(image_bytes)) as source:
                if not 0 < source.width * source.height <= MAX_CELLS:
                    raise ValueError("地图尺寸过大")
                image = grayscale(source, metadata)
            return self._commit(
                name,
                image,
                dict(
                    resolution=resolution,
                    origin=origin,
                    free_thresh=free,
                    occupied_thresh=occupied,
                    negate=metadata.get("negate", 0),
                    mode="trinary",
                ),
            )

    def list_routes(self, name):
        valid_name(name)
        path = self.routes / (name + ".json")
        return json.loads(path.read_text()) if path.exists() else {}

    def save_route(self, name, route_name, points):
        with self.lock:
            self.read(name)
            valid_name(route_name)
            if not isinstance(points, list) or not 1 <= len(points) <= 200:
                raise ValueError("路径需要 1–200 个目标点")
            routes = self.list_routes(name)
            routes[route_name] = [pose(point) for point in points]
            atomic_write(self.routes / (name + ".json"), json.dumps(routes).encode())

    def delete_route(self, name, route_name):
        with self.lock:
            routes = self.list_routes(name)
            if route_name not in routes:
                raise ValueError("路径不存在")
            del routes[route_name]
            atomic_write(self.routes / (valid_name(name) + ".json"), json.dumps(routes).encode())
