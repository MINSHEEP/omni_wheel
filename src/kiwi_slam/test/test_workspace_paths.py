"""默认 PCD 必须在工作区根 maps 内，不能随启动终端目录改变。"""

import pytest

from kiwi_slam.workspace_paths import default_map_pcd


@pytest.mark.parametrize("suffix", ["install/kiwi_slam", "install"])
def test_default_map_uses_workspace_root_with_both_install_layouts(tmp_path, monkeypatch, suffix):
    workspace = tmp_path / "robot"
    other = tmp_path / "other"
    other.mkdir()
    monkeypatch.chdir(other)
    result = default_map_pcd(workspace / suffix)
    assert result == workspace / "maps/mid360_map.pcd"
    assert result.parent.is_dir()
    assert not result.exists()


def test_custom_install_prefix_requires_explicit_pcd_path(tmp_path):
    with pytest.raises(ValueError, match="显式指定 map_pcd"):
        default_map_pcd(tmp_path / "custom_install")
