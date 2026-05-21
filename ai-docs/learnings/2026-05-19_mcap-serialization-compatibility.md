# MCAP Merge and ROS2 Compatibility Findings (Updated 2026-05-21)

## What changed
- The old robot source (`test-data/robot-v0_original`) was mixed serialization (`ros1` + `protobuf`) and required a pure MCAP-native merge workflow with non-ROS2 output expectations.
- The current robot source (`test-data/vps-v2_live-record-tests/rosbag2_2026_05_20-12_10_22`) is a true ROS2 bag in MCAP with `cdr` serialization.
- The current Fixposition source (`test-data/fixposition-remap-v8_mcap_bag`) is also ROS2 MCAP with `cdr` serialization.

## Current conclusion
- We still use MCAP CLI for filtering and merging because it provides direct topic filtering and deterministic file merge behavior.
- The merged result can be consumed by ROS2 tools after running `ros2 bag reindex` in the output folder.
- This is now the primary and only merge path for this project.

## Why MCAP CLI is still the primary tool
- `mcap filter` gives exact topic filtering in one step.
- `mcap merge` merges files by log timestamp and keeps sources untouched.
- In this workflow, ROS2 CLI is used for indexing and inspection (`ros2 bag reindex`, `ros2 bag info`) after merge.

## Important implementation details
- When merging two ROS2-generated MCAPs, duplicate metadata keys can occur (for example `rosbag2`).
- Use `mcap merge --allow-duplicate-metadata` to allow this.
- Keep the intermediate filtered MCAP outside the final bag folder. If it is inside, `ros2 bag info` will read both files and counts will appear doubled.

## Validated command pattern

Filter required Fixposition topics:

`mcap filter -o test-data/robot-v2_ros2-bag-tests/tmp/fixposition-selected_0.mcap -y '^/(camera/lowres/image_compressed|user_io/out/poi_locationfix|user_io/out/poi_odometry|user_io/out/poi_navsatfix|user_io/out/poi_pose)$' test-data/fixposition-remap-v8_mcap_bag/fixposition-v8_mcap_bag_0.mcap`

Merge into a new robot-v2 output bag:

`mcap merge --allow-duplicate-metadata -o test-data/robot-v2_ros2-bag-tests/rosbag2_2026_05_21-fixposition-merge/rosbag2_2026_05_21-fixposition-merge_0.mcap test-data/vps-v2_live-record-tests/rosbag2_2026_05_20-12_10_22/rosbag2_2026_05_20-12_10_22_0.mcap test-data/robot-v2_ros2-bag-tests/tmp/fixposition-selected_0.mcap`

Generate ROS2 metadata and validate:

`ros2 bag reindex test-data/robot-v2_ros2-bag-tests/rosbag2_2026_05_21-fixposition-merge`

`ros2 bag info test-data/robot-v2_ros2-bag-tests/rosbag2_2026_05_21-fixposition-merge`

## Result of latest run
- Final output folder: `test-data/robot-v2_ros2-bag-tests/rosbag2_2026_05_21-fixposition-merge`
- Final merged file: `rosbag2_2026_05_21-fixposition-merge_0.mcap`
- Required injected topics present:
	- `/camera/lowres/image_compressed`
	- `/user_io/out/poi_locationfix`
	- `/user_io/out/poi_odometry`
	- `/user_io/out/poi_navsatfix`
	- `/user_io/out/poi_pose`

---

Created: 2026-05-19
Updated: 2026-05-21
