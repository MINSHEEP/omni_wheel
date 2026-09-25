#!/usr/bin/env python3
"""Set the Kiwi ROS domain for new Bash terminals, without duplicate entries."""

import argparse
import shutil
from pathlib import Path


BEGIN = "# >>> Kiwi ROS 2 domain >>>"
END = "# <<< Kiwi ROS 2 domain <<<"


def main() -> None:
    parser = argparse.ArgumentParser(description="为新 Bash 终端设置 Kiwi 的 ROS_DOMAIN_ID")
    parser.add_argument("domain", nargs="?", type=int, default=42)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error("ROS_DOMAIN_ID 必须为 0–232")

    bashrc = Path.home() / ".bashrc"
    original = bashrc.read_text(encoding="utf-8") if bashrc.exists() else ""
    block = f"{BEGIN}\nexport ROS_DOMAIN_ID={args.domain}\n{END}\n"
    if BEGIN in original or END in original:
        if original.count(BEGIN) != 1 or original.count(END) != 1:
            parser.error(f"{bashrc} 中 Kiwi 域标记不完整或重复，请先检查文件")
        start = original.index(BEGIN)
        end = original.index(END, start) + len(END)
        if end < len(original) and original[end] == "\n":
            end += 1
        updated = original[:start] + block + original[end:]
    else:
        updated = block + original

    if updated != original:
        if bashrc.exists():
            backup = bashrc.with_name(".bashrc.kiwi-domain.bak")
            if not backup.exists():
                shutil.copy2(bashrc, backup)
        bashrc.write_text(updated, encoding="utf-8")
    print(f"{bashrc}: ROS_DOMAIN_ID={args.domain}；新终端生效，当前终端可运行 source ~/.bashrc")


if __name__ == "__main__":
    main()
