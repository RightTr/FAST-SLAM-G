# Waypoint Tools

Small ROS 2 helper scripts for recording, viewing, and running Nav2 waypoints.

## Record One Waypoint

Save the current robot pose from odometry into `points/waypoints.yaml`.

```bash
ros2 run fast_lio_sam_g waypoint_record.py --ros-args \
  -p name:=point_1
```

## Publish One Waypoint

Publish a saved waypoint to Nav2's goal topic.

```bash
ros2 run fast_lio_sam_g waypoint_publish.py --ros-args \
  -p waypoints_file:=/home/right/fastslam_g_ws/src/FAST-SLAM-G/waypoints/points/waypoints.yaml \
  -p topic:=/goal_pose \
  -p name:=point_1
```

## Show Waypoint Markers

Display saved waypoints in RViz.

```bash
ros2 run fast_lio_sam_g waypoint_markers.py --ros-args \
  -p waypoints_file:=/home/right/fastslam_g_ws/src/FAST-SLAM-G/waypoints/points/waypoints.yaml \
  -p marker_topic:=/waypoints_markers
```

## Record A Task Route

Record multiple goal poses into `waypoints/tasks/<task_name>.yaml`.

```bash
ros2 run fast_lio_sam_g waypoint_task_record.py --ros-args \
  -p task_name:=test_route \
  -p goal_pose_topic:=/goal_pose
```

Use RViz `2D Goal Pose` to add each waypoint.

## Run A Task Route

Run a saved task route with Nav2.

```bash
ros2 run fast_lio_sam_g waypoint_task_run.py --ros-args \
  -p task_name:=test_route \
  -p mode:=sequential
```

`mode:=sequential` sends one goal at a time. `mode:=through` sends all poses as one `NavigateThroughPoses` action.

## Task Dispatcher

The dispatcher listens for integer task IDs and runs mapped task files.

```bash
ros2 topic pub /waypoint_task_id std_msgs/msg/Int32 "{data: 1}" --once
```

Default task mappings are defined in `waypoint_task_dispatcher.py`.
