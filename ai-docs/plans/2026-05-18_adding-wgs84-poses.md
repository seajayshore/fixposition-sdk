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

## Phase 4 — WGS84 Pose (lat/lon/alt + ENU quaternion)

**Goal:** Derive position (lat/lon/alt) + orientation (quaternion in ENU frame) from the same ECEF odometry source.
1. **Primary output:** `geometry_msgs/msg/PoseStamped` (temporary mapping: x=lat [deg], y=lon [deg], z=alt [m])
2. **Foxglove output:** `foxglove_msgs/msg/LocationFix` from the same computed values (heading-only orientation), enabled when `foxglove_msgs` is available in the build environment

### Orientation derivation (reuse EcefPoseToEnuEul pattern)

Follow the pattern of [EcefPoseToEnuEul](fpsdk_common/src/trafo.cpp#L188) but keep quaternion output:

1. Extract ECEF quaternion from odometry and convert to rotation matrix:
  ```cpp
  const Eigen::Quaterniond q_ecef_body(odom.pose.pose.orientation.w,
                           odom.pose.pose.orientation.x,
                           odom.pose.pose.orientation.y,
                           odom.pose.pose.orientation.z);
  const Eigen::Matrix3d ecef_rot_matrix = q_ecef_body.normalized().toRotationMatrix();
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

### Heading extraction (for LocationFix)

For `LocationFix`, heading comes from ENU yaw:

```cpp
double yaw_enu = atan2(2.0 * (q_enu_body.w() * q_enu_body.z() + q_enu_body.x() * q_enu_body.y()),
                1.0 - 2.0 * (q_enu_body.y() * q_enu_body.y() + q_enu_body.z() * q_enu_body.z()));
double heading = M_PI_2 - yaw_enu;  // ENU yaw (CCW from East) -> compass heading (CW from North)
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

### Implementation approach

Add a single new static helper alongside `DeriveNavSatFix`. It should:
1. Detect ECEF odometry (same check as Phase 2: `|pos|² > (1e6)²`)
2. Reuse NavSatFix position derivation for lat/lon/alt
3. Compute ENU quaternion via the orientation derivation pattern above
4. Populate PoseStamped output topic
5. Return success/failure (same pattern as `DeriveNavSatFix`)

```cpp
static bool DeriveWgs84Pose(const nav_msgs::msg::Odometry& ecef_odom,
                             const std::string& src_topic,
                             geometry_msgs::msg::PoseStamped& pose_stamped,
                             std::string& pose_topic);
```

Wire in `WriteMessage()` after the existing NavSatFix derivation block.

### Files to change
- `fpsdk_ros2/src/bagwriter.cpp` — add includes, `DeriveWgs84Pose` helper, wire into `WriteMessage()`
- `fpsdk_ros2/test/bagwriter_test.cpp` — add tests for PoseStamped derivation

---

## Phase 5 — Build and verify (WGS84 Pose)

Same workflow as Phase 3:

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target fpltool -j$(nproc)

rm -rf test-data/v2-changes && mkdir -p test-data/v2-changes
./build/fpsdk_apps/fpltool rosbag test-data/fp-8f1240_2026-05-01-22-23-47_maximal.fpl \
  -o test-data/v2-changes/fp-8f1240_extract_bag
```

Verify:
- `_pose` topics exist and have type `geometry_msgs/msg/PoseStamped`
- Original `_odometry` topics are still `nav_msgs/msg/Odometry` (ECEF, unchanged)
- Original `_navsatfix` topics are still `sensor_msgs/msg/NavSatFix`
- Spot-check PoseStamped position mapping: `x≈latitude [deg]`, `y≈longitude [deg]`, `z≈altitude [m]` (matches NavSatFix)
- Spot-check PoseStamped orientation: quaternion changes smoothly and aligns with expected heading direction

---

## Relevant files
- [fpsdk_ros2/src/bagwriter.cpp](fpsdk_ros2/src/bagwriter.cpp) — primary edit target
- [fpsdk_ros2/include/fpsdk_ros2/bagwriter.hpp](fpsdk_ros2/include/fpsdk_ros2/bagwriter.hpp) — no change needed
- [fpsdk_common/include/fpsdk_common/trafo.hpp](fpsdk_common/include/fpsdk_common/trafo.hpp) — `TfWgs84LlhEcef`, `RotEnuEcef`, `LlhRadToDeg`, `TfEnuEcef`
- [fpsdk_ros2/CMakeLists.txt](fpsdk_ros2/CMakeLists.txt) — no changes required for Phase 4 Pose implementation
- [fpsdk_ros2/test/bagwriter_test.cpp](fpsdk_ros2/test/bagwriter_test.cpp) — add tests

---

## Decisions
- **Use [EcefPoseToEnuEul](fpsdk_common/src/trafo.cpp#L188) pattern for orientation:** Extract ECEF→ENU rotation via `RotEnuEcef(ecef_pos)`, apply to body rotation matrix, convert to quaternion. Skip Euler-angle conversion (unlike EcefPoseToEnuEul) — keep as quaternion for direct use in both outputs.
- **Position derivation:** PoseStamped uses the same position (lat/lon/alt) as NavSatFix Phase 2 logic. No ENU Cartesian needed here.
- **Heading from quaternion (deferred LocationFix):** Extract yaw via `atan2(2*(qw*qz+qx*qy), 1-2*(qy²+qz²))`, then convert to compass heading: `heading = π/2 - yaw_enu`.
- **PoseStamped hack:** Using x=lat, y=lon, z=alt (degrees/metres) is incorrect per message spec, but works for now. Proper solution would be `geographic_msgs/GeoPoseStamped` (future work).
- `DeriveNavSatFix` and `DeriveWgs84Pose` are file-scope static functions (not public API) — no header changes needed
- ECEF detection by position magnitude (`> 1e6 m`), not by topic name — robust to device config variations

---

## Visualisation in Foxglove — Research Findings

### Goal
Render the final position + heading on a 2D world map to validate that orientation (yaw/heading) is correct. Pitch and roll will not be visible in a 2D map view — that's fine.

### Primary: Foxglove Map panel + `foxglove_msgs/msg/LocationFix`

The Foxglove **Map panel** natively renders `foxglove_msgs/msg/LocationFix` with full heading support.

| Field | Mapping from Pose/NavSatFix derivations |
|---|---|
| `latitude`, `longitude`, `altitude` | Derived from NavSatFix conversion (Phase 2) |
| `heading` | Yaw from ENU quaternion: `heading = π/2 - yaw_enu` |
| `position_covariance` | Rotated ECEF→ENU covariance (from Phase 2) |

When `heading` is present, the Map panel automatically switches to **Arrowhead** point style — the marker rotates to point in the heading direction. This validates orientation visually.

### Current output: 3D visualization of PoseStamped

Foxglove **3D panel** can render `geometry_msgs/msg/PoseStamped` (from Phase 4a) as a 3D arrow or frame, showing the full quaternion at each timestep.

### Recommended workflow
1. Open Foxglove Studio with the extracted bag
2. Add a **Map panel**, subscribe to the `_locationfix` topic to inspect heading arrows
3. Add a **3D panel**, subscribe to the `_pose` topic to inspect quaternion orientation
4. Confirm PoseStamped position fields match NavSatFix values (`x=lat`, `y=lon`, `z=alt`)

If heading arrows align with the track's direction of motion, orientation is correct.
