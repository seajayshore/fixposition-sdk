# Fixposition -> Robot ROS2 MCAP Merge (Current Workflow)

This is the single-purpose command flow we use now.

Input robot bag:
- `test-data/vps-v2_live-record-tests/rosbag2_2026_05_20-12_10_22/rosbag2_2026_05_20-12_10_22_0.mcap`

Input Fixposition bag:
- `test-data/fixposition-remap-v8_mcap_bag/fixposition-v8_mcap_bag_0.mcap`

Final merged ROS2 bag folder:
- `test-data/robot-v2_ros2-bag-tests/rosbag2_2026_05_21-fixposition-merge`

## 1) Prerequisites

Install Homebrew (Linux):

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Add brew to shell startup if needed:

```bash
echo 'eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv bash)"' >> ~/.bashrc
source ~/.bashrc
```

Install MCAP CLI:

```bash
brew install mcap
```

Checks:

```bash
brew --version
mcap version
```

## 2) Prepare output directories

```bash
mkdir -p test-data/robot-v2_ros2-bag-tests/rosbag2_2026_05_21-fixposition-merge
mkdir -p test-data/robot-v2_ros2-bag-tests/tmp
```

## 3) Filter Fixposition topics

```bash
mcap filter \
  -o test-data/robot-v2_ros2-bag-tests/tmp/fixposition-selected_0.mcap \
  -y '^/(camera/lowres/image_compressed|user_io/out/poi_locationfix|user_io/out/poi_odometry|user_io/out/poi_navsatfix|user_io/out/poi_pose)$' \
  test-data/fixposition-remap-v8_mcap_bag/fixposition-v8_mcap_bag_0.mcap
```

## 4) Merge filtered Fixposition data into robot bag

```bash
mcap merge --allow-duplicate-metadata \
  -o test-data/robot-v2_ros2-bag-tests/rosbag2_2026_05_21-fixposition-merge/rosbag2_2026_05_21-fixposition-merge_0.mcap \
  test-data/vps-v2_live-record-tests/rosbag2_2026_05_20-12_10_22/rosbag2_2026_05_20-12_10_22_0.mcap \
  test-data/robot-v2_ros2-bag-tests/tmp/fixposition-selected_0.mcap
```

## 5) Generate ROS2 metadata.yaml

```bash
ros2 bag reindex test-data/robot-v2_ros2-bag-tests/rosbag2_2026_05_21-fixposition-merge
```

## 6) Validate

```bash
ros2 bag info test-data/robot-v2_ros2-bag-tests/rosbag2_2026_05_21-fixposition-merge
```

Required injected topics:
- `/camera/lowres/image_compressed`
- `/user_io/out/poi_locationfix`
- `/user_io/out/poi_odometry`
- `/user_io/out/poi_navsatfix`
- `/user_io/out/poi_pose`

Notes:
- Source bags are never modified.
- Keep intermediate filtered MCAP outside the final bag folder so `ros2 bag info` does not count it as part of the same bag.
