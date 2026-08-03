Import("env")

import os
import shutil

# The compression library (include/ + src/ at the repo root) is written once,
# for the host CMake build. PlatformIO expects library code under
# firmware/lib/<name>/{include,src}, so this pre-build hook copies the
# canonical source in fresh before every build. That makes the mirror a
# build artifact instead of a second hand-maintained copy -- there was
# previously a real drift risk here from manually re-copying files after
# every change to the canonical library.

FIRMWARE_DIR = env["PROJECT_DIR"]
REPO_ROOT = os.path.dirname(FIRMWARE_DIR)
CANONICAL_INCLUDE = os.path.join(REPO_ROOT, "include")
CANONICAL_SRC = os.path.join(REPO_ROOT, "src")
MIRROR_DIR = os.path.join(FIRMWARE_DIR, "lib", "occupancy_grid_lib")

# Host-only files that don't belong in the firmware library: main.cpp/
# benchmark.cpp/CMakeLists.txt are the host demo/benchmark/build entry
# points, not library code.
EXCLUDED = {"main.cpp", "benchmark.cpp", "CMakeLists.txt"}


def sync_dir(src_dir, dst_dir):
    if os.path.isdir(dst_dir):
        shutil.rmtree(dst_dir)
    os.makedirs(dst_dir, exist_ok=True)
    for name in os.listdir(src_dir):
        if name in EXCLUDED or name.startswith("."):
            continue
        src_path = os.path.join(src_dir, name)
        if os.path.isfile(src_path):
            shutil.copy2(src_path, os.path.join(dst_dir, name))


sync_dir(CANONICAL_INCLUDE, os.path.join(MIRROR_DIR, "include"))
sync_dir(CANONICAL_SRC, os.path.join(MIRROR_DIR, "src"))
print("[sync_lib] Synced occupancy_grid_lib from ../include and ../src")
