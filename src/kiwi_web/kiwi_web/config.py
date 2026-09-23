"""Portable workspace paths; each ROS package owns its parameter defaults."""

from pathlib import Path
from ament_index_python.packages import get_package_prefix

PARAMETER_FILES = (
    "base_params_file",
    "lidar_params_file",
    "slam_params_file",
    "mapping_gate_params_file",
    "monitor_params_file",
    "nav2_params_file",
)


def workspace_root(explicit="", prefix=None):
    if explicit:
        path = Path(explicit).expanduser()
        if not path.is_absolute():
            raise ValueError("workspace 必须是工作空间的绝对路径")
        return path.resolve()
    # Inspect the install prefix, not __file__: symlink-install points into src.
    installed = Path(prefix or get_package_prefix("kiwi_web")).absolute()
    if installed.name == "kiwi_web":
        installed = installed.parent
    if installed.name != "install":
        raise ValueError("非标准安装目录，请通过 --workspace 指定工作空间")
    return installed.parent.resolve()


def resolve_paths(config):
    root = workspace_root(config.get("workspace", ""))
    config["workspace"] = str(root)
    maps = Path(config.get("maps_dir") or "maps").expanduser()
    config["maps_dir"] = str((maps if maps.is_absolute() else root / maps).resolve())
    for key in (*PARAMETER_FILES, "platform_file"):
        value = config.get(key)
        if value:
            path = Path(value).expanduser()
            path = (path if path.is_absolute() else root / path).resolve()
            if not path.is_file():
                raise ValueError("参数文件不存在：%s=%s" % (key, path))
            config[key] = str(path)
    return config
