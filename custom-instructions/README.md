# Fixposition -> Robot MCAP Merge (Minimal Workflow)

This is the single-purpose command flow to produce a merged MCAP that keeps all robot data and adds selected Fixposition topics.

## 1) Prerequisites

### Install Homebrew (Linux)

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Add brew to your shell (if not already present):

```bash
echo 'eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv bash)"' >> ~/.bashrc
source ~/.bashrc
```

### Install MCAP CLI

```bash
brew install mcap
```

Sanity checks:

```bash
brew --version
mcap version
```

## 2) Build `fpltool`

From repo root:

```bash
cmake --build build --target fpsdk_ros2 fpltool -j"$(nproc)"
```

## 3) Export Fixposition data as MCAP

Use compression flags (`-c -c`) so ROS2 output is MCAP.

```bash
./build/fpsdk_apps/fpltool rosbag \
  test-data/fp-8f1240_2026-05-01-22-23-47_maximal.fpl \
  -c -c \
  -o test-data/fixposition-v8_mcap_bag/fixposition-v8_mcap
```

Expected output file:

```bash
test-data/fixposition-v8_mcap_bag/fixposition-v8_mcap_bag_0.mcap
```

Quick check:

```bash
mcap info test-data/fixposition-v8_mcap_bag/fixposition-v8_mcap_bag_0.mcap
```

## 4) Filter to only required Fixposition topics

```bash
mcap filter \
  -o test-data/robot-v1_merged/fixposition-filtered.mcap \
  -y '(/user_io/out/poi_navsatfix|/user_io/out/poi_locationfix|/camera/lowres/image_compressed)' \
  test-data/fixposition-v8_mcap_bag/fixposition-v8_mcap_bag_0.mcap
```

## 5) Merge with robot MCAP

```bash
mcap merge \
  -o test-data/robot-v1_merged/robot-fixposition-merged.mcap \
  test-data/robot-v0_original/2026-05-01_robot_ground-truth-test-1.mcap \
  test-data/robot-v1_merged/fixposition-filtered.mcap
```

## 6) Validate merged output

```bash
mcap info test-data/robot-v1_merged/robot-fixposition-merged.mcap
```

Confirm the merged bag includes:
- Full original robot topics
- `/user_io/out/poi_navsatfix`
- `/user_io/out/poi_locationfix`
- `/camera/lowres/image_compressed`

## Notes

- Source files are never modified.
- Intermediate file is `test-data/robot-v1_merged/fixposition-filtered.mcap`.
- The merged MCAP can contain mixed serialization formats and is intended for MCAP-native tools (for example Foxglove/rerun).
