# Plan: Adding WGS84 Poses from ECEF Odometry

**ECEF odometry topics (`poi_odometry`, `poi_smooth_odometry`) contain globally-referenced position and orientation in `nav_msgs/Odometry` format. This plan covers deriving WGS84/ENU outputs from these ECEF topics inside `BagWriter::WriteMessage()`. No changes needed to the extraction pipeline or `fpltool` itself.**

---

## Phase 1 — Create placeholder folders ✅ DONE
- Created `/ai-docs/plans/` and `/ai-docs/learnings/` directories in the repo

---

## Phase 2 — NavSatFix derivation in `bagwriter.cpp` ✅ DONE
*Changes in [fpsdk_ros2/src/bagwriter.cpp](fpsdk_ros2/src/bagwriter.cpp)*

- **Added includes**: `sensor_msgs/msg/nav_sat_fix.hpp` and `fpsdk_common/trafo.hpp`
- **Added static helper** `DeriveNavSatFix(odom, src_topic, fix, fix_topic) → bool`:
  - **ECEF detection**: `|pos|² > (1e6)²` — returns false if not ECEF
  - **Position**: `TfWgs84LlhEcef({x, y, z})` → lat/lon/alt via `LlhRadToDeg()`
  - **Covariance**: 3×3 top-left block from `odom.pose.covariance`, rotated to ENU: `C_enu = R * C_ecef * R^T` via `RotEnuEcef()`; `COVARIANCE_TYPE_KNOWN`
  - **Status**: `NavSatStatus::STATUS_FIX`
  - **Topic naming**: `_odometry` suffix → `_navsatfix` (guarded so NavSatFix can never land on the original odometry topic)
  - **Header**: copied from `odom.header`
- **Wired into `WriteMessage()`**: after `WriteMessageEx<nav_msgs::Odometry>` succeeds, calls `DeriveNavSatFix()` and writes NavSatFix to the derived topic
- **Tests added** in [fpsdk_ros2/test/bagwriter_test.cpp](fpsdk_ros2/test/bagwriter_test.cpp): ECEF odometry derives NavSatFix; non-ECEF odometry does not

---

## Phase 3 — Build and verify ✅ DONE
- Built with: `cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --target fpltool -j$(nproc)`
- Extracted: `./build/fpsdk_apps/fpltool rosbag test-data/fp-8f1240_2026-05-01-22-23-47_maximal.fpl -o test-data/v2-changes/fp-8f1240_extract_bag`
- Verified output bag contains `_navsatfix` and `_smooth_navsatfix` topics as `sensor_msgs/msg/NavSatFix`
- Verified original `_odometry` topics remain `nav_msgs/msg/Odometry`

---

## Phase 4 — WGS84 Poses: PoseStamped + LocationFix

**Goal:** Derive position (lat/lon/alt) + orientation (quaternion in ENU frame) from the same ECEF odometry source. Produce two complementary outputs:
1. **`/xsens/fusion`** — `geometry_msgs/msg/PoseStamped` with lat/lon/alt + ENU quaternion (currently using x=lat, y=lon, z=alt as a hack; could use `geographic_msgs/GeoPoseStamped` in future)
2. **`/xsens/fusion/locationfix`** — `foxglove_msgs/msg/LocationFix` with lat/lon/alt + heading (yaw only)

**Core relationship:** Both outputs share the same position (lat/lon/alt from NavSatFix derivation in Phase 2) and orientation source (ECEF quaternion from FP_A-ODOMETRY in the .fpl file). The difference is representation: PoseStamped keeps the full quaternion; LocationFix extracts only heading for map visualization.

### Orientation derivation (same source, two outputs)

Follow the pattern of [EcefPoseToEnuEul](fpsdk_common/src/trafo.cpp#L188) but produce a quaternion instead of Euler angles:

1. Extract rotation matrix from ECEF quaternion in the odometry:
   ```cpp
   const Eigen::Matrix3d ecef_rot_matrix = odom.pose.pose.orientation_as_matrix();
   ```
2. Rotate from ECEF frame to ENU frame (using position as reference):
   ```cpp
   const Eigen::Matrix3d R_enu_ecef = RotEnuEcef(ecef_pos);
   const Eigen::Matrix3d rot_enu_body = R_enu_ecef * ecef_rot_matrix;
   ```
3. Convert the rotated matrix back to quaternion:
   ```cpp
   const Eigen::Quaterniond q_enu_body(rot_enu_body);
   ```
4. Extract heading (yaw) from the ENU quaternion for LocationFix:
   ```cpp
   double yaw_enu = atan2(2.0 * (q_enu_body.w() * q_enu_body.z() + q_enu_body.x() * q_enu_body.y()),
                          1.0 - 2.0 * (q_enu_body.y() * q_enu_body.y() + q_enu_body.z() * q_enu_body.z()));
   double heading = M_PI_2 - yaw_enu;  // Convert ENU yaw (CCW from East) to compass heading (CW from North)
   ```

### 4a — PoseStamped topic (`/xsens/fusion`)

**Message type:** `geometry_msgs/msg/PoseStamped` (using x=lat, y=lon, z=alt hack)

**Fields:**
- `header.stamp` — from `odom.header.stamp`
- `header.frame_id` — from `odom.header.frame_id`
- `pose.position` — geographic coordinates (hack: x=lat [deg], y=lon [deg], z=alt [m])
  - Derived same as Phase 2 NavSatFix: `TfWgs84LlhEcef(ecef_pos)` → `LlhRadToDeg()`
- `pose.orientation` — quaternion in ENU frame (w, x, y, z)
  - Computed via orientation derivation steps above

**Required includes:** `#include <geometry_msgs/msg/pose_stamped.hpp>`

### 4b — LocationFix topic (`/xsens/fusion/locationfix`)

**Message type:** `foxglove_msgs/msg/LocationFix`

**Fields:**
- `timestamp` — from `odom.header.stamp`
- `frame_id` — from `odom.header.frame_id` (typically "" for GPS data)
- `latitude`, `longitude`, `altitude` — same as NavSatFix (Phase 2)
  - Reuse computation: `TfWgs84LlhEcef(ecef_pos)` → `LlhRadToDeg()` for lat/lon
- `position_covariance[9]` — from NavSatFix derivation (rotated ECEF→ENU covariance)
- `position_covariance_type` — `COVARIANCE_TYPE_KNOWN` (from Phase 2)
- `heading` — compass heading derived from ENU yaw (radians)
  - Computed via orientation derivation step 4 above

**Required includes:** `#include <foxglove_msgs/msg/location_fix.hpp>`

### Implementation approach

Add a single new static helper alongside `DeriveNavSatFix`. It should:
1. Detect ECEF odometry (same check as Phase 2: `|pos|² > (1e6)²`)
2. Reuse NavSatFix position derivation (lat/lon/alt, covariance rotation)
3. Compute ENU quaternion via the orientation derivation pattern above
4. Extract heading from the quaternion for LocationFix
5. Populate both PoseStamped and LocationFix outputs
6. Return success/failure (same pattern as `DeriveNavSatFix`)

```cpp
static bool DeriveWgs84Poses(const nav_msgs::msg::Odometry& ecef_odom,
                              const std::string& src_topic,
                              geometry_msgs::msg::PoseStamped& pose_stamped,
                              foxglove_msgs::msg::LocationFix& locationfix,
                              std::string& pose_topic,
                              std::string& locationfix_topic);
```

Wire in `WriteMessage()` after the existing NavSatFix derivation block.

### Files to change
- `fpsdk_ros2/src/bagwriter.cpp` — add includes, `DeriveWgs84Poses` helper, wire into `WriteMessage()`
- `fpsdk_ros2/CMakeLists.txt` — verify `foxglove_msgs` is available and linked
- `fpsdk_ros2/test/bagwriter_test.cpp` — add tests for PoseStamped and LocationFix derivation

---

## Phase 5 — Build and verify (WGS84 Poses)

Same workflow as Phase 3:

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target fpltool -j$(nproc)

rm -rf test-data/v2-changes && mkdir -p test-data/v2-changes
./build/fpsdk_apps/fpltool rosbag test-data/fp-8f1240_2026-05-01-22-23-47_maximal.fpl \
  -o test-data/v2-changes/fp-8f1240_extract_bag
```

Verify:
- `/xsens/fusion` topics exist and have type `geometry_msgs/msg/PoseStamped`
- `/xsens/fusion/locationfix` topics exist and have type `foxglove_msgs/msg/LocationFix`
- Original `_odometry` topics are still `nav_msgs/msg/Odometry` (ECEF, unchanged)
- Original `_navsatfix` topics are still `sensor_msgs/msg/NavSatFix`
- Spot-check PoseStamped: position near `(0, 0, 0)` for the first message, then drifts as vehicle moves (ENU local frame)
- Spot-check LocationFix: lat/lon/alt match NavSatFix values; heading points in direction of motion
- Open bag in Foxglove — add Map panel, subscribe to `_locationfix` topic (shows geographic track with heading arrows)

---

## Relevant files
- [fpsdk_ros2/src/bagwriter.cpp](fpsdk_ros2/src/bagwriter.cpp) — primary edit target
- [fpsdk_ros2/include/fpsdk_ros2/bagwriter.hpp](fpsdk_ros2/include/fpsdk_ros2/bagwriter.hpp) — no change needed
- [fpsdk_common/include/fpsdk_common/trafo.hpp](fpsdk_common/include/fpsdk_common/trafo.hpp) — `TfWgs84LlhEcef`, `RotEnuEcef`, `LlhRadToDeg`, `TfEnuEcef`
- [fpsdk_ros2/CMakeLists.txt](fpsdk_ros2/CMakeLists.txt) — verify `geometry_msgs` and `foxglove_msgs` linkage
- [fpsdk_ros2/test/bagwriter_test.cpp](fpsdk_ros2/test/bagwriter_test.cpp) — add tests

---

## Decisions
- **Use [EcefPoseToEnuEul](fpsdk_common/src/trafo.cpp#L188) pattern for orientation:** Extract ECEF→ENU rotation via `RotEnuEcef(ecef_pos)`, apply to body rotation matrix, convert to quaternion. Skip Euler-angle conversion (unlike EcefPoseToEnuEul) — keep as quaternion for direct use in both outputs.
- **Position derivation:** Both PoseStamped and LocationFix use the same position (lat/lon/alt) from NavSatFix Phase 2 logic. No ENU Cartesian needed here.
- **Heading from quaternion:** Extract yaw via `atan2(2*(qw*qz+qx*qy), 1-2*(qy²+qz²))`, then convert to compass heading: `heading = π/2 - yaw_enu`. This is the simple Z-axis rotation extraction.
- **PoseStamped hack:** Using x=lat, y=lon, z=alt (degrees/metres) is incorrect per message spec, but works for now. Proper solution would be `geographic_msgs/GeoPoseStamped` (future work).
- `DeriveNavSatFix` and `DeriveWgs84Poses` are file-scope static functions (not public API) — no header changes needed
- ECEF detection by position magnitude (`> 1e6 m`), not by topic name — robust to device config variations

---

## Visualisation in Foxglove — Research Findings

### Goal
Render the final position + heading on a 2D world map to validate that orientation (yaw/heading) is correct. Pitch and roll will not be visible in a 2D map view — that's fine.

### Primary: Foxglove Map panel + `foxglove_msgs/msg/LocationFix` ✅

The Foxglove **Map panel** natively renders `foxglove_msgs/msg/LocationFix` (produced by Phase 4b) with full heading support.

| Field | Mapping from Phase 4b |
|---|---|
| `latitude`, `longitude`, `altitude` | Derived from NavSatFix conversion (Phase 2) |
| `heading` | Yaw from ENU quaternion: `heading = π/2 - yaw_enu` |
| `position_covariance` | Rotated ECEF→ENU covariance (from Phase 2) |

When `heading` is present, the Map panel automatically switches to **Arrowhead** point style — the marker rotates to point in the heading direction. This validates orientation visually.

### Secondary: 3D visualization of PoseStamped

Foxglove **3D panel** can render `geometry_msgs/msg/PoseStamped` (from Phase 4a) as a 3D arrow or frame, showing the full pose (position + full quaternion) at each timestep. Useful for verifying pitch/roll if those are important for your application.

### Recommended workflow (once Phase 4 is complete)
1. Open Foxglove Studio with the extracted bag
2. Add a **Map panel**, subscribe to the `/xsens/fusion/locationfix` topic
3. Verify the track is visible and heading arrows point in direction of travel
4. Add a **3D panel** (optional), subscribe to `/xsens/fusion` to see full orientation

If heading arrows align with the track's direction of motion, orientation is correct.
