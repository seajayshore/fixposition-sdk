# MCAP Serialization Compatibility and Merge Findings (2026-05-19)

## Summary
- The robot's MCAP bag (`test-data/robot-v0_original/2026-05-01_robot_ground-truth-test-1.mcap`) uses `ros1` and `protobuf` serialization formats for all topics.
- The Fixposition MCAP bag (exported via fpltool) uses the `cdr` serialization format (the ROS2 default).
- Standard ROS2 tools (rosbag2_py, ros2 bag info, etc.) require all topics in a bag to use the same serialization format (typically `cdr`).
- Attempting to merge these bags with rosbag2_py fails due to serialization format mismatch.

## Implications
- The merged MCAP will not be fully compatible with ROS2 tools if it contains mixed serialization formats.
- Foxglove and rerun may still be able to open and visualize the merged MCAP, as they are more flexible with serialization formats.
- For true ROS2 compatibility, the robot bag would need to be converted to `cdr` serialization, which is non-trivial and not supported by standard tools.

## Recommended Approach
- For now, treat the robot's MCAP as the canonical/final format and append Fixposition topics as-is, even if serialization formats differ.
- Clearly document in the merge script and project notes that the merged MCAP is not ROS2-compliant and may not work with all tools.
- Add CLI warnings in the merge script if serialization format mismatches are detected, but allow the merge to proceed.

## Native MCAP CLI Workflow
- MCAP supports multiple serialization formats in a single file, so it is a better fit than rosbag2 for this merge task.
- The CLI includes a `filter` command for topic/time-range selection and a `merge` command that merges files by record timestamp.
- The simplest MCAP-native workflow is:
	1. Filter the Fixposition MCAP down to the three desired topics.
	2. Merge the filtered Fixposition MCAP with the robot MCAP into a new output file.
- The source bags remain untouched because both `filter` and `merge` write new files.
- This result is intended for Foxglove/rerun and custom analysis; do not expect ROS2 tooling to consume the merged file successfully.

## Practical Commands
- Install the MCAP CLI from the latest release binary or via Homebrew if available.
- Example topic filter step:
	`mcap filter fixposition-v8_mcap_bag/fixposition-v8_mcap_bag_0.mcap -o /tmp/fixposition_subset.mcap -y '^/user_io/out/poi_navsatfix$' -y '^/user_io/out/poi_locationfix$' -y '^camera/lowres/image_compressed$'`
- Example merge step:
	`mcap merge test-data/robot-v0_original/2026-05-01_robot_ground-truth-test-1.mcap /tmp/fixposition_subset.mcap -o test-data/robot-v1_merged.mcap`

## Next Steps
- Update the merge script to allow merging with mixed serialization formats, with clear warnings and comments.
- Document this limitation and the rationale in both the script and project documentation.

---

*Created: 2026-05-19*
