#!/usr/bin/env python3
"""拒绝缺少可跨动态库共享的 Gazebo 管理对象的二进制库。"""
from pathlib import Path
import subprocess
import sys


def check(directory):
    expected = {
        "libgazebo_transport.so.11": (
            "gazebo::transport::ConnectionManager",
            "gazebo::transport::TopicManager",
        ),
        "libgazebo_common.so.11": ("gazebo::common::ModelDatabase",),
    }
    missing = []
    for filename, classes in expected.items():
        library = directory / filename
        symbols = subprocess.run(
            ["readelf", "--dyn-syms", "--wide", str(library)],
            check=True, capture_output=True, text=True,
        ).stdout
        symbols = subprocess.run(
            ["c++filt"], input=symbols, check=True, capture_output=True, text=True,
        ).stdout
        objects = set()
        for line in symbols.splitlines():
            fields = line.split(None, 7)
            if len(fields) != 8:
                continue
            _, _, _, kind, binding, visibility, index, name = fields
            if (kind == "OBJECT" and binding in {"GLOBAL", "WEAK", "UNIQUE"}
                    and visibility == "DEFAULT" and index != "UND"):
                objects.add(name)
        for cls in classes:
            name = f"SingletonT<{cls}>::GetInstance()::t"
            if name in objects:
                print(f"共享对象导出通过：{cls}")
            else:
                missing.append(f"{filename}: {cls}")
    if missing:
        raise RuntimeError("缺少共享对象导出，不能继续部署：\n" + "\n".join(missing))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("用法：check_gazebo_singletons.py Gazebo动态库目录")
    try:
        check(Path(sys.argv[1]))
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(1)
