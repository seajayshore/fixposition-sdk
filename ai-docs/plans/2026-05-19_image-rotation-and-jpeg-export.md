# Plan: Image Rotation and JPEG Export for ROS2 Bag Extraction

**Goal:**
- Fix upside-down exported images by rotating 180° before writing to bag
- Add optional JPEG export (default) with raw fallback, CLI control, and topic suffixing
- Document and verify all steps with reproducible commands

---

## Phase 1 — 180° Image Rotation in Export ✅ DONE
- Added pure C++ helper for 180° rotation in [fpsdk_ros2/include/fpsdk_ros2/ros1.hpp]
- Hooked into ROS1→ROS2 image conversion so all exported images are now correctly oriented
- Preserved all metadata (header, encoding, step, etc.)
- Guarded against unsupported encodings/layouts (warn and pass-through)
- Verified with real bag extraction and visual check

## Phase 2 — Build, Smoke Test, and Document ✅ DONE
- Built with: `cmake --build build --target fpsdk_ros2 fpltool -j"$(nproc)"`
- Ran: `./build/fpsdk_apps/fpltool rosbag test-data/fp-8f1240_2026-05-01-22-23-47_maximal.fpl -o /tmp/fpsdk_phase1_rotate_check`
- Confirmed output bag contains correctly rotated images
- Documented working build/run flow in [ai-docs/learnings/2026-05-19_ros2-build-and-rosbag-workflow.md]

## Phase 3 — JPEG Export Mode, CLI, and Suffix ✅ DONE
- Added BagWriter config for image export mode (raw/jpeg) and JPEG quality
- Default is JPEG export to suffixed topic (`_compressed`)
- Raw mode available via CLI: `--ros-image-format raw`
- JPEG quality configurable: `--ros-image-jpeg-quality 90`
- Used OpenCV for JPEG encoding (with fallback to raw if unavailable)
- Updated CLI help and option parsing in fpltool
- Verified with:
  - `./build/fpsdk_apps/fpltool rosbag ...` (default: JPEG, suffixed topic, smaller bag)
  - `./build/fpsdk_apps/fpltool rosbag ... --ros-image-format raw` (original topic/type, larger bag)
  - Confirmed with `ros2 bag info` and file size difference
- Updated learning note with new CLI and results

## Phase 4 — Final Validation and Documentation ✅ DONE
- Rebuilt and ran both export modes on short sample
- Confirmed:
  - JPEG: `/camera/lowres/image_compressed` as `sensor_msgs/msg/CompressedImage`, bag size ~2.6 MiB
  - Raw: `/camera/lowres/image` as `sensor_msgs/msg/Image`, bag size ~13.7 MiB
- CLI help text shows new options
- All steps, commands, and caveats documented in [ai-docs/learnings/2026-05-19_ros2-build-and-rosbag-workflow.md]

---

## Outcomes
- All user-facing requirements for image orientation and export format are met
- Export is robust, CLI-configurable, and defaults to efficient JPEG
- Documentation and reproducibility are ensured for future work

---

**Next steps:**
- Move on to Docker/externalization and further workflow improvements in a new plan

---

*Created: 2026-05-19*
