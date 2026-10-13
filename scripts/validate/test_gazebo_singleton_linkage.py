"""用真实动态库复现发行补丁导致的管理对象分裂。"""
import ctypes
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
VENDOR = ROOT / "scripts/deploy/vendor/gazebo-source"


@unittest.skipUnless(shutil.which("g++") and shutil.which("patch"), "需要 GCC 和 patch")
class SingletonLinkageTests(unittest.TestCase):
    def addresses(self, root, patched):
        directory = root / ("patched" if patched else "upstream")
        header = directory / "gazebo/common/SingletonT.hh"
        header.parent.mkdir(parents=True)
        with tarfile.open(VENDOR / "gazebo_11.10.2+dfsg.orig.tar.xz") as archive:
            member = next(m for m in archive.getmembers()
                          if m.name.endswith("/gazebo/common/SingletonT.hh"))
            header.write_bytes(archive.extractfile(member).read())
        # SingletonT 本身不使用 system.hh 定义；可见性由测试类明确指定。
        system = directory / "gazebo/util/system.hh"
        system.parent.mkdir()
        system.write_text("\n")
        if patched:
            with tarfile.open(VENDOR / "gazebo_11.10.2+dfsg-1osrf6.debian.tar.xz") as archive:
                patch = archive.extractfile("debian/patches/0010-armhf-singleton.patch").read()
            subprocess.run(["patch", "--batch", "-p1"], cwd=directory, input=patch,
                           check=True, capture_output=True)
        source = directory / "probe.cc"
        source.write_text('''#include "gazebo/common/SingletonT.hh"
GZ_SINGLETON_DECLARE(__attribute__((visibility("default"))), xczs, audit, State)
namespace xczs { namespace audit {
class State : public SingletonT<State> {
  public: int value = 0;
};
}}
extern "C" __attribute__((visibility("default"))) void *state_address() {
  return xczs::audit::State::Instance();
}
''')
        addresses = []
        handles = []
        for name in ("first.so", "second.so"):
            library = directory / name
            subprocess.run(["g++", "-shared", "-fPIC", "-fvisibility=hidden",
                            "-fvisibility-inlines-hidden", "-I", str(directory),
                            str(source), "-o", str(library)], check=True, capture_output=True)
            handle = ctypes.CDLL(str(library), mode=os.RTLD_GLOBAL)
            handle.state_address.restype = ctypes.c_void_p
            addresses.append(handle.state_address())
            handles.append(handle)
        return addresses

    def test_original_declaration_shares_object_but_packaging_patch_splits_it(self):
        with tempfile.TemporaryDirectory(prefix="xczs-singleton-regression-") as directory:
            root = Path(directory)
            original = self.addresses(root, patched=False)
            broken = self.addresses(root, patched=True)
            self.assertEqual(original[0], original[1])
            self.assertNotEqual(broken[0], broken[1])


if __name__ == "__main__":
    unittest.main()
