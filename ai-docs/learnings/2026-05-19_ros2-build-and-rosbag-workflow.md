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

## 4) Phase 3 image export modes (working)

Default ROS2 image export is now JPEG on a suffixed topic:

```bash
./build/fpsdk_apps/fpltool rosbag test-data/fp-8f1240_2026-05-01-22-23-47_maximal.fpl -D 5 -o /tmp/fpsdk_phase3_jpeg_check
ros2 bag info /tmp/fpsdk_phase3_jpeg_check_bag
```

Observed result:
- Image topic: `/camera/lowres/image_compressed`
- Type: `sensor_msgs/msg/CompressedImage`
- Bag size for this short sample: about `2.6 MiB`

Explicit raw override:

```bash
./build/fpsdk_apps/fpltool rosbag test-data/fp-8f1240_2026-05-01-22-23-47_maximal.fpl -D 5 --ros-image-format raw -o /tmp/fpsdk_phase3_raw_check
ros2 bag info /tmp/fpsdk_phase3_raw_check_bag
```

Observed result:
- Image topic: `/camera/lowres/image`
- Type: `sensor_msgs/msg/Image`
- Bag size for this short sample: about `13.7 MiB`

Current CLI knobs:

```bash
--ros-image-format raw|jpeg
--ros-image-jpeg-quality 90
```

## Notes

- This machine can build and run the real extraction flow with CMake + existing `build/` tree.
- We intentionally did not gate this check on `fpsdk_ros2/test/bagwriter_test.cpp` because of known host prerequisite issues.
- JPEG export currently uses OpenCV when available and falls back to raw image export if JPEG encoding is unavailable.
