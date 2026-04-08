# Kalibr2

A modern multi-camera calibration toolbox built on ROS 2.

## Introduction

Kalibr2 is a camera calibration toolbox that provides:

- **Multi-Camera Calibration**: Intrinsic and extrinsic calibration of multi-camera systems with support for various camera models (pinhole, omni-directional, EUCM, double sphere)
- **ROS 2 Integration**: Native support for ROS 2
- **Modern C++**: Built with C++17 and modern CMake practices

This is a modernized version based on the original [Kalibr](https://github.com/ethz-asl/kalibr) calibration toolbox developed at ETH Zurich.

## Build it using docker

```bash
cd docker/
docker compose build kalibr_ros
docker compose run kalibr_ros
```

## Build it directly on Ubuntu 24.04 (noble)

### Prerequisites
- **ROS 2 Jazzy Desktop** - Install following the [official ROS 2 installation instructions](https://docs.ros.org/en/jazzy/Installation.html)

### Install System Dependencies
```bash
# Install all required packages
sudo apt update && sudo apt install -y $(cat ./packages-ubuntu-24.04.txt)
```

### Build
```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Usage

### Camera Calibration
```bash
# Calibrate cameras from a ROS 2 bag file
ros2 run kalibr2_ros kalibr_calibrate_cameras \
  --config path/to/calibration_config.yaml \
  --output-dir path/to/output/directory
```

### Calibration file example
The following example describes a calibration configuration for a stereo pair using the `pinhole` camera model with radial tangential distortion model.

It consumes a dataset from a recorded rosbag with an aprilgrid target of 6x5 tags and the corresponding sizes.

```yaml
board:
  target_type: 'aprilgrid' # grid type
  tagCols: 6               # number of apriltags in the x direction
  tagRows: 6               # number of apriltags in the y direction
  tagSize: 0.045           # size of apriltag, edge to edge [m]
  tagSpacing: 0.3          # ratio of space between tags to tagSize

cameras:
  camera_1_name:
    model: 'pinhole-radtan'
    focal_length_fallback: 881.0
    source:
      rosbag_path: '/path/to/your.mcap'
      topic: '/camera_1_topic/image'
  camera_2_topic:
    model: 'pinhole-radtan'
    focal_length_fallback: 881.0
    source:
      rosbag_path: '/path/to/your.mcap'
      topic: '/camera_2_topic/image'
```

### Output files
The calibration files are generated after a succesful run as follows.

#### Intrinsics
- `calibration_<camera_name>.yaml` - Camera intrinsics in ROS [CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html) format.

#### Extrinsics
- For 2-camera systems: `transform_<camera_0>_to_<camera_1>.yaml` - Baseline transform in ROS [TransformStamped](https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/TransformStamped.html) format.
- For multi-camera systems
`camera_chain_transforms.yaml` - Chain of baselines transforms in ROS [TFMessage](https://docs.ros.org/en/jazzy/p/tf2_msgs/msg/TFMessage.html) format.

## Extended parameters
There a number of configurations available in the tool that might come in handful.

```
kalibr_calibrate_cameras - Calibrate multiple cameras from ROS bag data
Usage: /home/frn/kalibr/install/kalibr2_ros/lib/kalibr2_ros/kalibr_calibrate_cameras [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -c,--config TEXT:FILE REQUIRED
                              Full path to calibration configuration YAML file.
  -o,--output-dir TEXT:DIR REQUIRED
                              Directory to save the calibration results.
  --approx-sync-tolerance FLOAT
                              Tolerance for approximate synchronization of observations across cameras (in seconds).
  --mi-tol FLOAT              The tolerance on the mutual information for adding an image. Higher means fewer images will be added. Use -1 to force all images.
  --max-batches UINT          Maximum number of batches to accept during incremental calibration. If not specified, all batches will be processed.
  --max-observations UINT     Maximum number of target observations to extract per camera. If not specified, all observations will be extracted.
  --verbose                   Enable verbose output during calibration.
```

## RGB-IMU Calibration (OAK-FFC-3P)

This section describes the full workflow to calibrate the **IMX577 RGB camera intrinsics**
and the **IMU-to-camera spatial and temporal extrinsics** for the OAK-FFC-3P.

Two tools are involved — record **one bag**, run them in sequence:

| Step | Tool | Package | What it produces |
|------|------|---------|-----------------|
| 1 | `kalibr_calibrate_cameras` | `kalibr2_ros` | Camera intrinsics (CameraInfo YAML) |
| 2 | `kalibr_calibrate_imu_camera` | `kalibr_imu_ros2` | IMU-camera rotation, translation, timeshift |

---

### Prerequisites

1. **Print the AprilGrid** defined in `kalibr_aprilgrid.yaml` (6×6, 45 mm tags, 0.3 spacing).
   Print on A3 at 100% scale, verify tag size with a ruler, mount flat on a rigid board.

2. **Source the workspace:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source /media/logic/USamsung/ros2_ws/install/setup.bash
   ```

3. **Create an output directory for calibration results:**
   ```bash
   mkdir -p /media/logic/USamsung/ros2_ws/src/kalibr_ros2/calibration_results
   mkdir -p /media/logic/USamsung/ros2_ws/calibration_bags
   ```

---

### Recording the Calibration Bag

Use the dedicated calibration driver config (`oak_ffc_3p_rgb_calibration.yaml`) which
records at **1920×1080 at 10 FPS** — not the 4K survey config.

> **Why not 4K?** The IMX577 produces ~25 MB per 4K frame. Calibration accuracy is
> determined by corner detection quality, not resolution. 1920×1080 gives identical results
> with ~4× smaller bags and significantly faster processing.

**Terminal 1 — Launch the driver:**
```bash
cd /media/logic/USamsung/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=$(pwd)/src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_rgb_calibration.yaml \
  camera_model:=OAK-FFC-3P
```

**Terminal 2 — Record the bag (60–90 seconds):**
```bash
cd /media/logic/USamsung/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 bag record -o calibration_bags/rgb_imu_calibration \
  /oak/rgb/image_raw \
  /oak/imu/data
```

**Movement procedure:**
- Mount the AprilGrid flat on a wall — **keep the target stationary, move the camera rig**
- Move slowly and smoothly (avoid motion blur at 10 FPS)
- Cover all four corners and edges of the image frame
- Tilt and rotate — vary roll, pitch, and yaw relative to the target
- Translate left/right, up/down, toward/away to excite all IMU axes
- Aim for 60–90 seconds of continuous motion

Stop recording with Ctrl-C in Terminal 2, then Ctrl-C in Terminal 1.

---

### Step 1: Camera Intrinsics

Calibrates focal length, principal point, and equidistant distortion coefficients.

```bash
cd /media/logic/USamsung/ros2_ws
ros2 run kalibr2_ros kalibr_calibrate_cameras \
  --config src/kalibr_ros2/aslam_offline_calibration/kalibr2_ros/kalibr_rgb_config.yaml \
  --output-dir src/kalibr_ros2/calibration_results/
```

The config (`kalibr_rgb_config.yaml`) is pre-configured for the IMX577:

- `model: 'pinhole-radtan'` — radial-tangential model for the corrected wide-angle lens
- `focal_length_fallback: 806.0` — initial guess: `fx = 1920 / (2 × tan(50°)) ≈ 806 px`

> **Why `pinhole-radtan`, not `pinhole-equi`?**  
> Despite the 113° diagonal FOV, the M12 HQ113 lens spec states **Distortion: <-1.5%**.
> Fisheye lenses at this FOV have 30–50%+ distortion. A corrected wide-angle lens with 1.5%
> distortion behaves like a rectilinear projection and is correctly modelled with `radtan`.
> Using `pinhole-equi` here causes the optimizer to diverge (~485 px RMSE). Basalt also
> confirms `radtan` is correct — it uses `pinhole-radtan8` for this lens successfully.

> **Do I need to provide HFOV or the DepthAI extrinsics JSON?**  
> No. Only `focal_length_fallback` is needed as an initial guess — Kalibr optimises it from
> the data. The DepthAI calibration JSON is not used. No camera prior beyond the focal length
> estimate is required.

Output file (CameraInfo format):
```
src/kalibr_ros2/calibration_results/calibration_rgb_imx577.yaml
```

**Quality check:** Reprojection error should be **< 0.5 px**. Above 1 px indicates a
problem — re-record with slower movement and better corner coverage.

To limit processing time on long bags:
```bash
ros2 run kalibr2_ros kalibr_calibrate_cameras \
  --config src/kalibr_ros2/aslam_offline_calibration/kalibr2_ros/kalibr_rgb_config.yaml \
  --output-dir src/kalibr_ros2/calibration_results/ \
  --max-observations 500
```

---

### Step 2: Create the Camchain YAML

The IMU-camera tool (`kalibr_calibrate_imu_camera`) requires camera parameters in a
**camchain format** that differs from the CameraInfo output of Step 1. Create this file
manually by copying values from `calibration_rgb_imx577.yaml`.

**Extract values from the Step 1 output:**
```
k: [fx, 0,  cx,
    0,  fy, cy,
    0,  0,  1 ]
```
- `fx` = `k[0]`, `fy` = `k[4]`, `cx` = `k[2]`, `cy` = `k[5]`
- `distortion_coeffs` = the 4-element `d` array (`[k1, k2, k3, k4]`)

**Create** `src/kalibr_ros2/calibration_results/camchain_rgb_imx577.yaml`:

```yaml
cam0:
  camera_model: pinhole            # always 'pinhole' for this tool
  distortion_model: equidistant    # matches 'pinhole-equi' used in Step 1
  intrinsics: [FX, FY, CX, CY]    # fill from calibration_rgb_imx577.yaml k matrix
  distortion_coeffs: [K1, K2, K3, K4]  # fill from d[] in Step 1 output (4 values)
  resolution: [1920, 1080]
  rostopic: /oak/rgb/image_raw
  cam_overlaps: []
```

> **Note the naming difference:**  
> Step 1 config uses `model: 'pinhole-equi'` (C++ format).  
> Step 2 camchain uses `camera_model: pinhole` + `distortion_model: equidistant` (Python format).  
> These are the same model expressed differently — both tools must see consistent values.

---

### Step 3: IMU-Camera Extrinsics

Calibrates the rigid-body transform (rotation + translation) from the IMX577 camera frame
to the BMI270 IMU frame, plus the temporal offset between their clocks.

> **Do I need to provide the IMU orientation as a prior?**  
> No. Kalibr estimates the full IMU-to-camera rotation from the motion data in the bag.
> No orientation prior, no extrinsics guess, and no DepthAI JSON are needed.

```bash
cd /media/logic/USamsung/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
ros2 run kalibr_imu_ros2 kalibr_calibrate_imu_camera \
  --bag calibration_bags/rgb_imu_calibration/rgb_imu_calibration_0.mcap \
  --cam src/kalibr_ros2/calibration_results/camchain_rgb_imx577.yaml \
  --imu src/kalibr_ros2/kalibr_bmi270_imu.yaml \
  --target src/kalibr_ros2/kalibr_aprilgrid.yaml \
  --dont-show-report \
  --num-threads 4 2>&1
```

Output files are written **alongside the bag directory** (no `--output-dir` flag):
```
calibration_bags/rgb_imu_calibration-camchain-imucam.yaml   ← T_cam_imu + timeshift
calibration_bags/rgb_imu_calibration-imu.yaml               ← refined IMU parameters
calibration_bags/rgb_imu_calibration-results-imucam.txt     ← text summary
calibration_bags/rgb_imu_calibration-report-imucam.pdf      ← visual report
```

**Quality targets** (check `*-results-imucam.txt`):
- Reprojection error: < 1.0 px (< 0.5 px is excellent)
- `|timeshift_cam_imu|`: typically < 10 ms for USB3 hardware sync
- Gyroscope residual RMS: < 0.01 rad/s

**Optional — downsample extraction rate** on very long bags (speeds up the AprilGrid
detection pass):
```bash
  --bag-freq 4     # extract grid detections at 4 Hz instead of full 10 Hz
```

---

### Step 4: Apply Results to survey_ros2

```bash
cd /media/logic/USamsung/ros2_ws
python3 src/survey_ros2/scripts/apply_kalibr_calibration.py \
  --intrinsics src/kalibr_ros2/calibration_results/calibration_rgb_imx577.yaml \
  --camchain   calibration_bags/rgb_imu_calibration-camchain-imucam.yaml \
  --output     src/survey_ros2/config/camera_calibration.yaml
```

Pass both `--intrinsics` and `--camchain` together to update intrinsics and IMU-camera
extrinsics in one step. Either argument can be omitted to update only one set of values.

---

### Key Files Reference

| File | Purpose |
|------|---------|
| `kalibr_aprilgrid.yaml` | AprilGrid target definition (shared by both tools) |
| `kalibr_bmi270_imu.yaml` | BMI270 noise params; `rostopic: /oak/imu/data`; `update_rate: 200` |
| `depthai_ros_driver/config/oak_ffc_3p_rgb_calibration.yaml` | Driver config for recording (1920×1080, 10 FPS, IMU 200 Hz, no YOLO) |
| `aslam_offline_calibration/kalibr2_ros/kalibr_rgb_config.yaml` | Step 1 input: `pinhole-equi`, `focal_length_fallback: 635.0` |
| `calibration_results/calibration_rgb_imx577.yaml` | Step 1 output: CameraInfo intrinsics |
| `calibration_results/camchain_rgb_imx577.yaml` | Step 2 input (created manually): camchain format for IMU tool |
| `<bag>-camchain-imucam.yaml` | Step 3 output: T_cam_imu rotation, translation, timeshift |
| `survey_ros2/config/camera_calibration.yaml` | Final destination updated by `apply_kalibr_calibration.py` |

---

### Troubleshooting

**`Unknown camera model: pinhole-equidistant`** — The C++ tool only accepts `'pinhole-equi'`
(not the longer spelling). Check `kalibr_rgb_config.yaml`.

**Reprojection error > 1 px in Step 1** — Re-record the bag. Common causes: motion blur
(move more slowly at 10 FPS), insufficient corner coverage (visit all image corners and
edges), or the target was not printed at the correct scale (re-measure the tag size).

**`distortion_model 'equidistant' requires 4 coefficients`** — When building
`camchain_rgb_imx577.yaml`, verify `distortion_coeffs` has exactly 4 values from the `d`
field in `calibration_rgb_imx577.yaml`.

**IMU calibration diverges / does not converge** — The noise values in `kalibr_bmi270_imu.yaml`
are from the BMI270 datasheet (minimum bounds). Real-world values depend on PCB layout and
temperature. If calibration fails to converge, run an Allan Variance analysis on a 30-minute
static bag and replace the noise density values with measured results.

---

## References

The calibration approaches in Kalibr2 are based on the original Kalibr toolbox. Please cite the appropriate papers when using this toolbox in academic work:

1. Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
2. Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.

## Original Kalibr Authors
- Paul Furgale
- Hannes Sommer
- Jérôme Maye
- Jörn Rehder
- Thomas Schneider
- Luc Oth

## License (BSD)
Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland<br>
Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland<br>
All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

1. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

1. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

1. Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL the AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
