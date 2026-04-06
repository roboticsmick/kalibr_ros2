#!/usr/bin/env python3
import os
import re
import subprocess
import xml.etree.ElementTree as ET


def print_header(msg):
    print(f"\n{'='*60}\n[CHECK] {msg}\n{'='*60}")


def find_packages_and_orphans():
    valid_packages = []
    orphans = []

    for dirpath, dirnames, filenames in os.walk("."):
        # Skip build/install/log directories
        if any(
            ignored in dirpath for ignored in ["/build", "/install", "/log", "/.git"]
        ):
            continue

        # Respect COLCON_IGNORE — prune this dir and all subdirs
        if "COLCON_IGNORE" in filenames:
            dirnames.clear()
            continue

        has_cmake = "CMakeLists.txt" in filenames
        has_xml = "package.xml" in filenames
        has_setup = "setup.py" in filenames

        if has_xml and (has_cmake or has_setup):
            valid_packages.append(dirpath)
        elif has_cmake and not has_xml:
            # We skip standard cmake sub-directories like 'cmake' folders
            if not dirpath.endswith("/cmake"):
                orphans.append(dirpath)

    return valid_packages, orphans


def parse_cmake(filepath):
    with open(filepath, "r") as f:
        content = f.read()
    # Find all find_package(NAME ...) — include hyphens for packages like yaml-cpp
    matches = re.findall(r"find_package\s*\(\s*([a-zA-Z0-9_\-]+)", content)
    return set(matches)


def parse_package_xml(filepath):
    tree = ET.parse(filepath)
    root = tree.getroot()
    deps = set()
    for tag in ["depend", "build_depend", "build_export_depend", "exec_depend"]:
        for elem in root.findall(f".//{tag}"):
            deps.add(elem.text.strip())
    return deps


def check_orphans(orphans):
    print_header("Step 1: Checking for 'Orphan' CMake projects")
    if orphans:
        print(
            "❌ WARNING: The following directories have a CMakeLists.txt but NO package.xml!"
        )
        print(
            "   Colcon will completely IGNORE these folders, causing linking errors later."
        )
        for orphan in orphans:
            print(f"   - {orphan}")
        print("\n-> FIX: You must add a minimal package.xml to these folders,")
        print("   or merge them into a parent package.")
    else:
        print("✅ No orphaned CMake projects found. Colcon sees everything.")


def cross_check_packages(valid_packages):
    print_header("Step 2: Cross-Checking CMake vs package.xml for all packages")
    all_good = True
    ignore_list = {"Threads", "OpenGL", "Doxygen", "ament_cmake", "Python3", "catkin",
                   "ament_lint_auto", "ament_lint_common", "ament_cmake_python",
                   "rosidl_default_generators", "rosidl_default_runtime"}

    # Map CMake find_package() names -> canonical rosdep keys used in package.xml
    cmake_to_rosdep = {
        "Boost":       "boost",
        "Eigen3":      "eigen",
        "SuiteSparse": "suitesparse",
        "OpenCV":      "libopencv-dev",
        "yaml-cpp":    "yaml-cpp",
    }

    for pkg in valid_packages:
        cmake_path = os.path.join(pkg, "CMakeLists.txt")
        xml_path = os.path.join(pkg, "package.xml")

        if not os.path.exists(cmake_path):
            continue  # Skip pure python packages

        cmake_raw = parse_cmake(cmake_path) - ignore_list
        # Translate cmake names to their rosdep equivalents for comparison
        cmake_deps = {cmake_to_rosdep.get(d, d) for d in cmake_raw}

        xml_deps = parse_package_xml(xml_path)

        missing_in_xml = cmake_deps - xml_deps
        if missing_in_xml:
            all_good = False
            print(f"❌ [{pkg}] Missing in package.xml:")
            for dep in missing_in_xml:
                print(f"     - Add <depend>{dep}</depend>")

    if all_good:
        print("✅ All package.xml files match their CMakeLists.txt requirements.")


def run_rosdep():
    print_header("Step 3: Checking System Dependencies (rosdep)")
    result = subprocess.run(
        ["rosdep", "check", "--from-paths", ".", "--ignore-src"],
        capture_output=True,
        text=True,
    )
    if (
        "System dependencies have not been satisfied" in result.stderr
        or result.returncode != 0
    ):
        print("❌ WARNING: Missing system dependencies!")
        print(result.stderr or result.stdout)
        print("-> Run: rosdep install --from-paths . --ignore-src -r -y")
    else:
        print("✅ All system libraries are installed.")


def run_colcon_dry_run():
    print_header("Step 4: Colcon CMake Configuration Dry-Run")
    print("Running CMake Configure for the whole workspace (Skipping C++ compile)...")

    # --cmake-target help forces CMake to configure the tree but skips compiling the code!
    cmd = [
        "colcon",
        "build",
        "--cmake-target",
        "help",
        "--event-handlers",
        "console_direct+",
    ]

    # Run from the directory ABOVE src/kalibr_ros2 (the workspace root)
    # If we are in src/kalibr_ros2, we need to go up two levels conceptually,
    # but running it here with --base-paths . works too.
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print("❌ CMake Configuration FAILED.")
        print(
            "Look for the 'Failed' package below to see where your links are broken:\n"
        )
        # Extract just the errors to keep output clean
        lines = result.stderr.split("\n") + result.stdout.split("\n")
        for line in lines:
            if (
                "CMake Error" in line
                or "Could NOT find" in line
                or "Failed   <<<" in line
            ):
                print(line)
    else:
        print("✅ All packages configured successfully! Links are valid.")


if __name__ == "__main__":
    valid_pkgs, orphans = find_packages_and_orphans()

    check_orphans(orphans)
    cross_check_packages(valid_pkgs)
    run_rosdep()

    print(
        "\n⚠️  Note for Step 4: Make sure you run this script from the root of your ros2_ws"
    )
    print("   so colcon works properly.\n")

    # We only run colcon if they are in the workspace root or src folder
    if os.path.exists("src"):
        run_colcon_dry_run()
    else:
        print("⏭️  Skipping Step 4 (Colcon Dry Run). Go to your ros2_ws root and run:")
        print("   colcon build --cmake-target help")
