# 2026-05-19: ROS2 Build and Rosbag Workflow (Known Good)

This note captures the commands that worked on this machine while implementing ROS2 image rotation in bag export.

## 1) Build targets with CMake (working)

Run from repository root:

```bash
cmake --build build --target fpsdk_ros2 fpltool -j"$(nproc)"
```

Observed result:
- `Built target fpsdk_ros2`
- `Built target fpltool`

## 2) Run rosbag export smoke test (working)

```bash
./build/fpsdk_apps/fpltool rosbag test-data/fp-8f1240_2026-05-01-22-23-47_maximal.fpl -o /tmp/fpsdk_phase1_rotate_check
```

Observed result:
- Extraction completed successfully
- Output bag directory: `/tmp/fpsdk_phase1_rotate_check_bag`
- Reported size: about 3962.6 MiB

## 3) Verify output bag metadata/topics (working)

```bash
ros2 bag info /tmp/fpsdk_phase1_rotate_check_bag
```

Observed result includes:
- Storage id: `sqlite3`
- Camera topic preserved: `/camera/lowres/image` with type `sensor_msgs/msg/Image`
- Derived odometry topics still present (`_navsatfix`, `_locationfix`, etc.)

## Notes

- This machine can build and run the real extraction flow with CMake + existing `build/` tree.
- We intentionally did not gate this check on `fpsdk_ros2/test/bagwriter_test.cpp` because of known host prerequisite issues.
- Use this as the baseline workflow for upcoming image export changes (JPEG phase).
