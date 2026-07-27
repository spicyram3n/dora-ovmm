#!/bin/bash
# Idempotent build fixes for the upstream packages postCreate.sh clones.
# Hand-editing the checkouts instead does not survive a workspace rebuild.
set -e

ROS2_WS="${1:-/home/ws/ros2_ws}"
SRC="$ROS2_WS/src"

# an sed whose anchor moved upstream changes nothing and fails the build later
require_file() {
    [ -f "$1" ] || { echo "bug_fixes.sh: expected file is missing: $1" >&2; exit 1; }
}

assert_patched() {
    grep -qF "$2" "$1" || {
        echo "bug_fixes.sh: patch did NOT apply to $1 ($3) - upstream file changed?" >&2
        exit 1
    }
    echo "bug_fixes.sh: patched $1 ($3)"
}

# hsrb_moveit_config: missing find_package()s break linking the demo executables
moveit_config_cmake="$SRC/hsrb_moveit/hsrb_moveit_config/CMakeLists.txt"
require_file "$moveit_config_cmake"
if ! grep -qF "find_package(geometric_shapes REQUIRED)" "$moveit_config_cmake"; then
    sed -i \
        -e '/^find_package(Eigen3 REQUIRED)$/a\
        find_package(Boost REQUIRED COMPONENTS random)\
        find_package(geometric_shapes REQUIRED)\
        add_link_options("-Wl,--no-as-needed")' \
        -e '/^ament_target_dependencies/ s/rclcpp)$/rclcpp geometric_shapes)/' \
        "$moveit_config_cmake"
    assert_patched "$moveit_config_cmake" "find_package(geometric_shapes REQUIRED)" \
        "Boost::random / geometric_shapes"
fi

# tmc_robot_kinematics_model: on newer pinocchio ${pinocchio_LIBRARIES} is empty
# and ${pinocchio_INCLUDE_DIRS} misses the deprecated-header dir
rkm_cmake="$SRC/tmc_manipulation_base/tmc_robot_kinematics_model/CMakeLists.txt"
require_file "$rkm_cmake"
if ! grep -qF "pinocchio::pinocchio" "$rkm_cmake"; then
    sed -i \
        -e 's|target_include_directories(pinocchio_wrapper PUBLIC include \${pinocchio_INCLUDE_DIRS} \${EIGEN3_INCLUDE_DIRS})|target_include_directories(pinocchio_wrapper PUBLIC include ${EIGEN3_INCLUDE_DIRS})|' \
        -e 's|target_link_libraries(pinocchio_wrapper \${pinocchio_LIBRARIES})|target_link_libraries(pinocchio_wrapper pinocchio::pinocchio)|' \
        -e 's|ament_export_include_directories(include \${pinocchio_INCLUDE_DIRS})|ament_export_include_directories(include ${pinocchio_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS}/pinocchio/deprecated)|' \
        "$rkm_cmake"
    assert_patched "$rkm_cmake" "pinocchio::pinocchio" "pinocchio::pinocchio target"
fi

# tmc_robot_kinematics_model: pinocchio's JointCollectionDefault exceeds
# boost::mpl's default 20-type list
rkm_header="$SRC/tmc_manipulation_base/tmc_robot_kinematics_model/include/tmc_robot_kinematics_model/pinocchio_wrapper.hpp"
require_file "$rkm_header"
if ! grep -qF "BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS" "$rkm_header"; then
    python3 - "$rkm_header" <<'EOF'
import sys
path = sys.argv[1]
marker = "#include <pinocchio/multibody/model.hpp>"
guard = """// Raising the list size is a no-op unless NO_PREPROCESSED_HEADERS is set too,
// and both must precede the first transitive boost include.
#ifndef BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif
#ifndef BOOST_MPL_LIMIT_LIST_SIZE
#define BOOST_MPL_LIMIT_LIST_SIZE 30
#endif
#ifndef BOOST_MPL_LIMIT_VECTOR_SIZE
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30
#endif

"""
text = open(path).read()
assert marker in text, f"marker not found in {path}"
open(path, "w").write(text.replace(marker, guard + marker, 1))
EOF
    assert_patched "$rkm_header" "BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS" "boost::mpl limits"
fi

# tmc_robot_rrt_planner_node: pinocchio leaves Boost_USE_DEBUG_RUNTIME=TRUE set,
# which breaks the later Boost::random lookup for geometric_shapes
rrt_cmake="$SRC/tmc_manipulation_planner/tmc_robot_rrt_planner_node/CMakeLists.txt"
require_file "$rrt_cmake"
if ! grep -qF "Boost_USE_DEBUG_RUNTIME" "$rrt_cmake"; then
    sed -i \
        -e '/find_package(ament_cmake_gtest REQUIRED)/a\
  # reset: pinocchio leaves this TRUE, which hides the release Boost::random\
  set(Boost_USE_DEBUG_RUNTIME OFF)' \
        "$rrt_cmake"
    assert_patched "$rrt_cmake" "Boost_USE_DEBUG_RUNTIME" "Boost_USE_DEBUG_RUNTIME"
fi

# tmc_robot_collision_detector: sets no C++ standard, but the
# tmc_collision_detector header it includes declares std::optional
rcd_cmake="$SRC/tmc_manipulation_base/tmc_robot_collision_detector/CMakeLists.txt"
require_file "$rcd_cmake"
if ! grep -qF "CMAKE_CXX_STANDARD" "$rcd_cmake"; then
    sed -i \
        -e '/^project(tmc_robot_collision_detector)$/a\
\
set(CMAKE_CXX_STANDARD 17)\
set(CMAKE_CXX_STANDARD_REQUIRED ON)' \
        "$rcd_cmake"
    assert_patched "$rcd_cmake" "set(CMAKE_CXX_STANDARD 17)" "C++17 for std::optional"
fi

# tmc_gazebo_worlds: the env hook only sets GAZEBO_MODEL_PATH (Gazebo Classic),
# so on Ignition every model:// URI in the world fails to resolve
worlds_hook="$SRC/tmc_gazebo/tmc_gazebo_worlds/env-hooks/tmc_gazebo_worlds.dsv.in"
require_file "$worlds_hook"
if ! grep -qF "GZ_SIM_RESOURCE_PATH" "$worlds_hook"; then
    printf 'prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share/@PROJECT_NAME@/models\nprepend-non-duplicate;IGN_GAZEBO_RESOURCE_PATH;share/@PROJECT_NAME@/models\n' \
        >> "$worlds_hook"
    assert_patched "$worlds_hook" "GZ_SIM_RESOURCE_PATH" "Ignition model resource path"
fi
