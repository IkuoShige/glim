# glim_ros 環境構築ガイド

このドキュメントはglim_rosとEPCD拡張を別環境で再現するための完全な手順書。
TBBあり/なし両方の構成を記載。

## 動作確認済み環境

| 項目 | 値 |
|---|---|
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| Kernel | 6.8.0-106-generic |
| Compiler | GCC 11.4.0 |
| CMake | 3.22.1 |
| C++ Standard | C++17 |

## 1. システム依存パッケージ (apt)

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake git \
  libeigen3-dev \
  libboost-all-dev \
  libopencv-dev \
  libspdlog-dev \
  libfmt-dev \
  libpcl-dev \
  libomp-dev \
  libtbb-dev \
  libmetis-dev \
  libglfw3-dev \
  libglew-dev \
  libgl1-mesa-dev \
  libopengl-dev
```

各バージョン:

| パッケージ | バージョン |
|---|---|
| Eigen | 3.4.0 |
| Boost | 1.74.0 |
| OpenCV | 4.5.4 |
| spdlog | 1.9.2 |
| fmt | 8.1.1 |
| PCL | 1.12.1 |
| OpenMP (libomp) | 14.0 |
| TBB | 2021.5.0 |
| GLFW/GLEW | 3.3.6 / 2.2.0 |

## 2. ROS 2 Humble

```bash
# ROS 2 Humble (Ubuntu 22.04)
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-rosbag2 \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions
```

## 3. GTSAM 4.3a0

### TBBなし (推奨、Threadripper等の多コアCPU向け)

```bash
cd /root
git clone --depth 1 --branch 4.3a0 https://github.com/borglab/gtsam.git
cd gtsam && mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=ON \
  -DGTSAM_POSE3_EXPMAP=ON \
  -DGTSAM_ROT3_EXPMAP=ON \
  -DGTSAM_SUPPORT_NESTED_DISSECTION=ON \
  -DGTSAM_TANGENT_PREINTEGRATION=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

### TBBあり (Intel N150等の少コアCPU向け)

```bash
cd /root/gtsam/build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGTSAM_WITH_TBB=ON \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=ON \
  -DGTSAM_POSE3_EXPMAP=ON \
  -DGTSAM_ROT3_EXPMAP=ON \
  -DGTSAM_SUPPORT_NESTED_DISSECTION=ON \
  -DGTSAM_TANGENT_PREINTEGRATION=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

**切替時の注意**: TBBあり/なしを切り替える場合は、GTSAM → gtsam_points → glim の順で全て再ビルドが必要。ABI互換性がないため部分再ビルドは不可。

### 確認方法

```bash
grep "GTSAM_USE_TBB\|GTSAM_ALLOCATOR" /usr/local/include/gtsam/config.h
```

TBBなしの場合:
```
/* #undef GTSAM_USE_TBB */
#define GTSAM_ALLOCATOR_STL
```

TBBありの場合:
```
#define GTSAM_USE_TBB
#define GTSAM_ALLOCATOR_TBB
```

## 4. gtsam_points 1.2.0

```bash
cd /root
git clone https://github.com/koide3/gtsam_points.git
cd gtsam_points
git checkout 11af1e6  # v1.2.0相当
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_WITH_OPENMP=ON \
  -DBUILD_WITH_TBB=OFF \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

**TBBあり構成の場合**: `-DBUILD_WITH_TBB=ON` に変更。GTSAMのTBB設定と一致させること。

**CUDA有効の場合**: `-DBUILD_WITH_CUDA=ON` に変更 (NVIDIA GPU + CUDA toolkit必要)。

### 確認方法

```bash
grep "GTSAM_POINTS_USE_TBB\|GTSAM_POINTS_USE_OPENMP" /usr/local/include/gtsam_points/config.hpp
```

## 5. Iridescence 0.1.6 (viewer用)

```bash
cd /root
git clone https://github.com/koide3/iridescence.git
cd iridescence && mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

viewerが不要な場合は省略可 (`-DBUILD_WITH_VIEWER=OFF` でglimをビルド)。

## 6. glim + glim_ros ワークスペース

```bash
mkdir -p /root/ros2_ws/src && cd /root/ros2_ws/src
# glim と glim_ros2 を配置 (git clone or コピー)
# glim/ と glim_ros2/ がsrc/直下にある前提

cd /root/ros2_ws
source /opt/ros/humble/setup.bash
```

### ビルド (viewerあり、Release)

```bash
colcon build --packages-up-to glim_ros --symlink-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_VIEWER=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DBUILD_TESTING=ON
```

### ビルド (headless、テスト用)

```bash
colcon build --packages-up-to glim_ros --symlink-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_VIEWER=OFF \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DBUILD_TESTING=ON
```

### テスト

```bash
source install/setup.bash
colcon test --packages-select glim \
  --event-handlers console_direct+ \
  --ctest-args -R test_exact_gicp_factor --output-on-failure
```

## 7. 評価ツール

```bash
pip3 install evo numpy scipy
```

| ツール | バージョン | 用途 |
|---|---|---|
| evo | 1.35.2 | ATE (Absolute Trajectory Error) 算出 |
| numpy | 2.2.6 | evo依存 |
| scipy | 1.15.3 | evo依存 |

## 8. EPCD評価の実行

```bash
cd /root/ros2_ws
source install/setup.bash

# 自動評価スクリプト
./src/glim/scripts/evaluate_epcd.sh \
  /root/ros2_ws/src/glim/config/epcd_livox_budget \
  /tmp/glim_eval_output

# 手動実行
ros2 run glim_ros glim_rosbag \
  /root/ros2_ws/src/outdoor_hard_01a \
  /root/ros2_ws/src/outdoor_hard_01b \
  --ros-args \
  -p config_path:=/root/ros2_ws/src/glim/config/epcd_livox_budget \
  -p dump_path:=/tmp/glim_eval_output \
  -p auto_quit:=true

# ATE算出
evo_ape tum \
  /root/ros2_ws/src/gt/traj_lidar_outdoor_hard_01.txt \
  /tmp/glim_eval_output/traj_lidar.txt \
  --align
```

## 9. 設定プロファイル一覧

| プロファイル | 用途 | TBB | スレッド |
|---|---|---|---|
| `config/epcd_livox_cpu` | 標準EPCD (96pt, 15kf, 8iter) | 不問 | 4 |
| `config/epcd_livox_budget` | **推奨** (96pt, 10kf, 6iter) | なし推奨 | 4 |
| `config/epcd_livox_n150` | N150用 (= budgetと同一) | あり推奨 | 4 |
| `config/epcd_livox_budget_mt` | 多コアCPU用 (16threads) | なし | 16 |
| `config/livox_gpu_headless` | GPU baseline比較用 | 不問 | - |

## 10. TBBあり/なし切替手順

### TBBなし → TBBあり

```bash
# 1. GTSAM再ビルド
cd /root/gtsam/build
cmake .. -DGTSAM_WITH_TBB=ON  # 他のフラグはそのまま
make -j$(nproc) && sudo make install && sudo ldconfig

# 2. gtsam_points再ビルド
cd /root/gtsam_points/build
cmake .. -DBUILD_WITH_TBB=ON  # 他のフラグはそのまま
make -j$(nproc) && sudo make install && sudo ldconfig

# 3. glim再ビルド
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to glim_ros --symlink-install \
  --cmake-args -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TESTING=ON

# 4. テスト確認
colcon test --packages-select glim --event-handlers console_direct+ \
  --ctest-args -R test_exact_gicp_factor --output-on-failure
```

### TBBあり → TBBなし

同じ手順で `-DGTSAM_WITH_TBB=OFF`, `-DBUILD_WITH_TBB=OFF` に変更。

### 確認

```bash
# GTSAMのTBB状態
grep "GTSAM_USE_TBB" /usr/local/include/gtsam/config.h

# gtsam_pointsのTBB状態
grep "GTSAM_POINTS_USE_TBB" /usr/local/include/gtsam_points/config.hpp

# glimがリンクしているライブラリ
ldd /root/ros2_ws/install/glim/lib/libglim.so | grep tbb
```

## 11. 実測ベンチマーク (参考値)

環境: AMD Ryzen Threadripper 3970X 32-Core, 64GB RAM
データ: outdoor_hard_01a/b (682.3s, 5140 frames)

| 構成 | TBB | Threads | Wall time | RT倍率 | ATE RMSE |
|---|---|---|---|---|---|
| EPCD budget | OFF | 4 | **456s** | **1.50x** | **1.82m** |
| EPCD budget | ON | 4 | 542s | 1.26x | 2.06m |
| EPCD budget + IIEC | OFF | 4 | 619s | 1.10x | 2.20m |
| EPCD budget + IIEC | ON | 4 | 613s | 1.11x | 2.00m |
| GPU GLIM baseline | - | - | 60s | 11.46x | 1.99m |

**推奨**: 多コアCPU (>8コア) → TBBなし。少コアCPU (4コア, N150) → TBBあり (未検証、論文主張に基づく)。
