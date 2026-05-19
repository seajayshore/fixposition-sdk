# Plan: MCAP Export and Cross-Bag Merge (Completed)


Build this in three separable phases: first make the Fixposition export reliably produce MCAP so the source side is stable, then use the native MCAP CLI (`filter` + `merge`) to combine the untouched robot MCAP with only the selected Fixposition topics into a fresh output bag, then build and test the merge workflow end to end. **Note:** Due to serialization format differences (robot: ros1/protobuf, Fixposition: cdr), the merged MCAP will not be ROS2-compliant but will be suitable for Foxglove/rerun and custom analysis tools. See ai-docs/learnings/2026-05-19_mcap-serialization-compatibility.md for details.

## Final Status
- [x] Workflow validated end to end using MCAP CLI.
- [x] Source bags left untouched.
- [x] Final merged output created at `test-data/robot-v1_merged/robot-fixposition-merged.mcap`.
- [x] Python merge prototype retired in favor of native MCAP tooling.

## Phase 1: Lock Down Fixposition MCAP Export
- [x] Confirm the current ROS2 export path produces MCAP for the Fixposition dataset and that the default bag contents remain the expected derived topics.
- [x] Verify the export includes the three needed additions for later merge work: `/user_io/out/poi_navsatfix`, `/user_io/out/poi_locationfix`, and `camera/lowres/image_compressed`.
- [x] Keep this phase limited to export format, topic naming, and runtime validation so the merge work starts from a known-good source bag.
- [x] Treat this as the prerequisite gate before any merge scripting begins.

## Phase 2: Prototype the MCAP-Native Merge Path
- [x] Create a standalone script or utility that reads the robot MCAP as the immutable base and produces a new merged MCAP rather than modifying either source. (Completed with MCAP CLI command workflow.)
- [x] Prefer the MCAP CLI/native library path over rosbag2 tools, because MCAP supports heterogeneous serialization formats in a single file and the robot bag already uses a mixed format.
- [x] Filter the Fixposition bag down to only `/user_io/out/poi_navsatfix`, `/user_io/out/poi_locationfix`, and `camera/lowres/image_compressed` before merging.
- [x] Merge the filtered Fixposition MCAP into the robot MCAP by record timestamp.
- [x] Keep the source bags untouched and use explicit CLI/script warnings when the merged result is expected to be non-ROS2-compliant.
- [x] If needed for local iteration, place the prototype in a clearly disposable, gitignored scratch area inside this repo while keeping the long-term target external.


## Phase 3: Build, Test, and Harden the Merge Workflow (Foxglove/rerun Focus)
- [x] Run the native MCAP workflow against the sample robot MCAP and the Fixposition-derived MCAP, then verify the merged output contains the full robot dataset plus the three added topics.
- [x] The merge result is intentionally allowed to contain mixed serialization formats (robot: ros1/protobuf, Fixposition: cdr) and should be treated as a Foxglove/rerun-oriented artifact, not a ROS2 playback bag.
- [x] Validate topic names, message types, and counts with MCAP inspection tools and Foxglove/rerun.
- [x] Check for timestamp alignment issues between the two datasets and decide whether exact stamp matching is sufficient or whether a fixed offset/sync rule is needed. (Current policy: exact timestamp merge, no fixed offset.)
- [x] Capture the final command sequence, assumptions, and any caveats in `ai-docs/learnings` so the eventual out-of-repo implementation can be recreated cleanly.

---

**Relevant files**
- `fpsdk_ros2/src/bagwriter.cpp` — Current source of truth for Fixposition-derived topic generation and export behavior.
- `fpsdk_ros2/include/fpsdk_ros2/bagwriter.hpp` — Shows the existing image-export controls and defaults used by the Fixposition bag writer.
- `fpsdk_apps/fpltool/fpltool_opts.hpp` — Documents the current ROS image export CLI options.
- `fpsdk_apps/fpltool/fpltool_extract.cpp` — Shows how export configuration is passed into bag writing.
- `test-data/robot-v0_original/2026-05-01_robot_ground-truth-test-1.mcap` — Robot source bag for merge validation.
- `test-data/fixposition-v8_mcap_bag` — Fixposition-derived MCAP source bag used for merge validation.
- `ai-docs/learnings` — Place for commands, findings, and caveats discovered while building the merge path.

---


**Verification**
- [x] Confirm the Fixposition export produces MCAP and the expected target topics before starting any merge work.
- [x] Run the merge prototype on short sample data and verify that the merged output contains all robot topics plus `/user_io/out/poi_navsatfix`, `/user_io/out/poi_locationfix`, and `camera/lowres/image_compressed`.
- [x] Inspect the merged bag with MCAP tooling and Foxglove/rerun to confirm topic types and message counts. (Do not expect ROS2 tools to work on the merged bag.)
- [x] Check whether the two source datasets require any clock offset or other synchronization adjustment.
- [x] Record the validated commands and any skipped or deferred checks in `ai-docs/learnings`.

---

**Decisions**
- Phase 1 is strictly about making the Fixposition side MCAP-ready and verifying the needed topics are present.
- The merge output should be a brand-new bag so the original robot data stays untouched.
- Use MCAP CLI commands directly as the primary implementation (no custom merge script required).
- Included scope: export format confirmation, bag-level merge orchestration, topic filtering, and validation.
- Excluded scope: changing the robot data producer, rewriting Fixposition derivation logic inside the merge tool, and broader downstream processing.

---

**Further Considerations**
- Merge location: Option A prototype inside a gitignored scratch folder in this repo. Option B start in a separate repo immediately. Recommendation: Option A for faster iteration, with a clear migration path to an external repo once the approach is proven.
- Alignment policy: Option A exact timestamp matching only. Option B allow a fixed offset between the two bags. Recommendation: start with Option A during the prototype, then add offset support only if the sample data shows it is necessary.
