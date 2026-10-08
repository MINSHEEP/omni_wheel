"""运行时地图位于安装前缀所属的工作区，兼容隔离与合并安装。"""

from pathlib import Path

from ament_index_python.packages import get_package_prefix


def default_map_pcd(prefix=None):
    # 不使用 __file__：symlink-install 时模块来自 src，而非安装前缀。
    installed = Path(prefix or get_package_prefix("kiwi_slam")).absolute()
    if installed.name == "kiwi_slam":
        installed = installed.parent
    if installed.name != "install":
        raise ValueError("非标准安装目录，请显式指定 map_pcd 绝对路径")
    maps = installed.parent.resolve() / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    return maps / "mid360_map.pcd"
