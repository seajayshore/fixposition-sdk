# Plan: Add NavSatFix Derivation from ECEF Odometry

**ECEF odometry topics (`poi_odometry`, `poi_smooth_odometry`) contain globally-referenced position in `nav_msgs/Odometry` format. We add a NavSatFix derivation step inside `BagWriter::WriteMessage()` — after writing the ECEF odometry as-is, we also compute and write a `sensor_msgs/NavSatFix` topic. No changes needed to the extraction pipeline or `fpltool` itself.**

---

## Phase 1 — Create placeholder folders (trivial, first step)
1. Create `/ai-docs/plans/` and `/ai-docs/learnings/` directories in the repo

---

## Phase 2 — NavSatFix derivation in `bagwriter.cpp`
*All changes confined to [fpsdk_ros2/src/bagwriter.cpp](fpsdk_ros2/src/bagwriter.cpp) unless noted*

2. **Add includes**: `sensor_msgs/msg/nav_sat_fix.hpp` and `fpsdk_common/trafo.hpp`

3. **Add static helper function** `DeriveNavSatFix(const nav_msgs::msg::Odometry& odom, const std::string& src_topic, sensor_msgs::msg::NavSatFix& fix, std::string& fix_topic) → bool`:
   - **ECEF detection**: check `|pos|² > (1e6)²` — ENU0 positions are near-zero so this cleanly distinguishes them; return false if not ECEF
   - **Position**: `TfWgs84LlhEcef({x, y, z})` → `[lat_rad, lon_rad, alt_m]`, convert to degrees via `LlhRadToDeg()`
   - **Covariance**: extract the 3×3 top-left block from `odom.pose.covariance[36]` (ECEF XX/YY/ZZ/XY/XZ/YZ), rotate to ENU frame: `C_enu = R * C_ecef * R^T` where `R = RotEnuEcef(ecef_vec)`; fill `fix.position_covariance[9]` (row-major E/N/U order); set `position_covariance_type = COVARIANCE_TYPE_KNOWN`
   - **Status**: `fix.status.status = NavSatStatus::STATUS_FIX` (best default available; RTK quality is not encoded in Odometry)
   - **Topic naming**: replace trailing `_odometry` suffix with `_navsatfix` (e.g. `/user_io/out/poi_odometry` → `/user_io/out/poi_navsatfix`)
   - **Header**: copy `odom.header` directly (same frame_id, same timestamp)

4. **Wire into `WriteMessage()`**: after the `WriteMessageEx<nav_msgs::Odometry, nav_msgs::msg::Odometry>` branch succeeds, call `DeriveNavSatFix()` and if it returns true, `WriteMessage<sensor_msgs::msg::NavSatFix>(fix, fix_topic, rosmsgbin.rec_time_)`

---

## Phase 3 — Build and verify
5. `colcon build --packages-select fpsdk_common fpsdk_ros2 fpsdk_apps` on Ubuntu 22.04 / ROS2 Humble
6. Run the locally built `fpltool rosbag test-data/fp-8f1240_2026-05-01-22-23-47_maximal.fpl` against the test `.fpl` file
7. Verify output bag contains `/user_io/out/poi_navsatfix` and `/user_io/out/poi_smooth_navsatfix`; spot-check lat/lon values are in range (approximately Switzerland based on the device name suggesting Fixposition origin)
8. Validate with `./fpsdk.sh` Docker workflow (rebuild or bind-mount local build)

---

## Phase 4 — Future (separate session)
9. ENU/WGS84 `nav_msgs/Odometry` (position + orientation both expressed in WGS84 ENU frame, derived from the same ECEF topics). Orientation conversion requires rotating the quaternion from ECEF frame to local ENU frame using `RotEnuEcef()`.

---

## Relevant files
- [fpsdk_ros2/src/bagwriter.cpp](fpsdk_ros2/src/bagwriter.cpp) — primary edit target; add include, helper, wire into `WriteMessage()`
- [fpsdk_ros2/include/fpsdk_ros2/bagwriter.hpp](fpsdk_ros2/include/fpsdk_ros2/bagwriter.hpp) — no change needed (helper stays in `.cpp`)
- [fpsdk_common/include/fpsdk_common/trafo.hpp](fpsdk_common/include/fpsdk_common/trafo.hpp) — reference for `TfWgs84LlhEcef`, `RotEnuEcef`, `LlhRadToDeg`
- [fpsdk_ros2/CMakeLists.txt](fpsdk_ros2/CMakeLists.txt) — check `sensor_msgs` and `Eigen3` linkage (likely no change needed)
- [fpsdk_ros2/test/bagwriter_test.cpp](fpsdk_ros2/test/bagwriter_test.cpp) — add test for NavSatFix derivation

---

## Decisions
- `DeriveNavSatFix` is a file-scope static function (not a public API) — no header changes needed
- ECEF detection by position magnitude (`> 1e6 m`), not by topic name — more robust to device config variations
- `STATUS_FIX` used for NavSatFix status (RTK quality not available in `nav_msgs/Odometry`)
- Covariance is properly rotated ECEF→ENU (not just diagonals), using `RotEnuEcef`

---

## Further considerations
1. The `poi_odometry` frame_id (set by device firmware) will be copied into the NavSatFix header. We should verify at implementation time what string the device uses (`ros2 topic echo` on the bag or `sqlite3` query) — if it's `earth` or `ECEF`, that's technically wrong for NavSatFix (which should have `""` or a GPS frame). We can strip/override it.
2. `sensor_msgs::msg::NavSatFix` has no velocity field — the full position+orientation ENU Odometry (Phase 4) will cover the remaining data from these topics.
