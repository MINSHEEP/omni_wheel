"""Bounded PCD validation and durable, no-overwrite map copies."""

import errno
import os
from pathlib import Path
import shutil
import stat
import struct
import tempfile

from .maps import valid_name


MAX_HEADER_BYTES = 64 * 1024
MAX_ASCII_ROW_BYTES = 64 * 1024
ENCODINGS = {"ascii", "binary", "binary_compressed"}


def point_cloud_path(root, name):
    """Return a confined PCD destination for a MapStore-compatible name."""
    target = Path(root).expanduser().resolve() / (valid_name(name) + ".pcd")
    if target.is_symlink():
        raise ValueError("不支持符号链接点云地图")
    return target


def _open_regular(path):
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        if error.errno == errno.ELOOP:
            raise ValueError("不支持符号链接点云地图") from error
        raise
    if not stat.S_ISREG(os.fstat(descriptor).st_mode):
        os.close(descriptor)
        raise ValueError("PCD 来源必须是普通文件")
    return os.fdopen(descriptor, "rb")


def _header(stream):
    fields = {}
    consumed = 0
    while consumed < MAX_HEADER_BYTES:
        line = stream.readline(MAX_HEADER_BYTES - consumed + 1)
        if not line:
            break
        consumed += len(line)
        if consumed > MAX_HEADER_BYTES:
            break
        try:
            entry = line.decode("ascii").strip().split()
        except UnicodeDecodeError as error:
            raise ValueError("PCD 头部不是 ASCII 文本") from error
        if not entry or entry[0].startswith("#"):
            continue
        key = entry[0].upper()
        if key in fields:
            raise ValueError("PCD 头部字段重复：" + key)
        fields[key] = entry[1:]
        if key == "DATA":
            return fields, stream.tell()
    raise ValueError("PCD 头部缺少 DATA 或超过长度限制")


def _single(fields, key):
    values = fields.get(key, ())
    if len(values) != 1:
        raise ValueError("PCD 头部缺少或无效：" + key)
    return values[0]


def _positive_integer(value, field):
    try:
        number = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError("PCD 头部数字无效：" + field) from error
    if number <= 0:
        raise ValueError("PCD 头部数字须大于零：" + field)
    return number


def validate_pcd(path):
    """Check PCD structure with bounded memory; return its point count and size.

    The compressed mode checks stored byte lengths, not LZF contents. This is
    an integrity guard for a newly written file, not geometric validation.
    """
    with _open_regular(path) as stream:
        file_size = os.fstat(stream.fileno()).st_size
        fields, data_offset = _header(stream)
        names = fields.get("FIELDS") or fields.get("FIELD")
        if not names or not {"x", "y", "z"}.issubset(names):
            raise ValueError("PCD 必须包含 x/y/z 坐标字段")
        sizes = fields.get("SIZE", ())
        types = fields.get("TYPE", ())
        counts = fields.get("COUNT", ["1"] * len(names))
        if not len(names) == len(sizes) == len(types) == len(counts):
            raise ValueError("PCD 字段、类型、长度或数量不匹配")
        if any(kind not in ("F", "I", "U") for kind in types):
            raise ValueError("PCD 字段类型无效")
        stride = sum(
            _positive_integer(size, "SIZE") * _positive_integer(count, "COUNT")
            for size, count in zip(sizes, counts)
        )
        width = _positive_integer(_single(fields, "WIDTH"), "WIDTH")
        height = _positive_integer(_single(fields, "HEIGHT"), "HEIGHT")
        points = _positive_integer(_single(fields, "POINTS"), "POINTS")
        if width * height != points:
            raise ValueError("PCD 点数与宽高不一致")
        encoding = _single(fields, "DATA")
        if encoding not in ENCODINGS:
            raise ValueError("不支持的 PCD 数据格式")
        remaining = file_size - data_offset
        if encoding == "binary":
            if remaining != points * stride:
                raise ValueError("PCD 二进制数据长度与点数不一致")
        elif encoding == "binary_compressed":
            if remaining < 8:
                raise ValueError("PCD 压缩数据不完整")
            compressed, uncompressed = struct.unpack("<II", stream.read(8))
            if compressed == 0 or remaining != compressed + 8 or uncompressed != points * stride:
                raise ValueError("PCD 压缩数据长度与点数不一致")
        else:
            rows = 0
            expected_fields = sum(map(int, counts))
            while line := stream.readline(MAX_ASCII_ROW_BYTES + 1):
                if len(line) > MAX_ASCII_ROW_BYTES:
                    raise ValueError("PCD ASCII 数据行过长")
                if line.strip():
                    rows += 1
                    if rows > points or len(line.split()) != expected_fields:
                        raise ValueError("PCD ASCII 数据与点数或字段不一致")
            if rows != points:
                raise ValueError("PCD ASCII 数据与点数或字段不一致")
        return {"points": points, "encoding": encoding, "size_bytes": file_size}


def copy_validated_pcd(source, root, name):
    """Validate and atomically install a PCD alongside a named 2D map.

    The destination is never replaced, even if a concurrent save uses the same
    name. The source must be complete and no longer written by the producer.
    """
    target = point_cloud_path(root, name)
    if target.exists():
        raise ValueError("同名点云地图已存在，请使用新名称保存")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with _open_regular(source) as input_file:
            before = os.fstat(input_file.fileno())
            descriptor, temporary = tempfile.mkstemp(
                prefix="." + target.name + ".", suffix=".tmp", dir=target.parent
            )
            with os.fdopen(descriptor, "wb") as output:
                shutil.copyfileobj(input_file, output, length=1024 * 1024)
                output.flush()
                os.fsync(output.fileno())
            after = os.fstat(input_file.fileno())
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise ValueError("PCD 来源在复制过程中发生变化，请重试")
        metadata = validate_pcd(temporary)
        try:
            os.link(temporary, target)
        except FileExistsError as error:
            raise ValueError("同名点云地图已存在，请使用新名称保存") from error
        directory = os.open(target.parent, os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
        return metadata
    finally:
        if temporary is not None:
            Path(temporary).unlink(missing_ok=True)
