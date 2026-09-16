# Why this package is vendored

MoveIt 2.5.10's `moveit_simple_controller_manager`, unchanged except for one
thing: `FollowJointTrajectoryControllerHandle::sendTrajectory` waits a bounded
5 s for the controller's goal response and resends up to three times instead of
blocking forever. On the real HSR a goal request occasionally never reaches the
robot's controller (docs/realrobot_grasp_test.md, problem 9). The apt package is
still installed; this overlay's plugin shadows it once `ros2_ws/install` is
sourced. Rebuild with:

    cd ros2_ws && colcon build --symlink-install --packages-select moveit_simple_controller_manager \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
