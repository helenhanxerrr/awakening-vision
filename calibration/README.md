# 相机拍照与标定

移植自 [同济 sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25/tree/bd9f5e798fa3c6dd3b483ae6627796afb41c608d/calibration)，基准提交 `bd9f5e798fa3c6dd3b483ae6627796afb41c608d`，保留 MIT 许可于 `LICENSE-TongjiSuperPower`。
拍照程序使用武科现有 C++ 相机工厂与串口驱动；离线解算用 Python/OpenCV 4 实现同济的圆点板内参、Tsai 手眼、Shah 机器人世界／手眼联合标定流程。另提供原配套录像裁剪功能。

## 构建与安装

以下命令从仓库根目录执行。拍照端部署于 Linux（与现有武科驱动一致）；离线标定可在 Windows/Linux 上运行。

```bash
# 独立构建拍照工具，不依赖 ROS2、推理模型、OpenVINO 或 TensorRT。
sudo apt install cmake make g++ libopencv-dev libeigen3-dev libyaml-cpp-dev libfmt-dev \
  libspdlog-dev libtbb-dev libboost-system-dev libceres-dev
cmake -S calibration -B build/calibration
cmake --build build/calibration -j2

# 在独立 Python 环境安装 OpenCV 4（OpenCV 5 接口不同）。
python3 -m venv .venv-calibration
. .venv-calibration/bin/activate
python -m pip install -r calibration/requirements.txt
```

海康／迈德威视／大恒需要先安装对应 SDK，并使用项目 `cmake/Find*SDK.cmake` 规定的路径。海康设置 `MVCAM_SDK_PATH`。没有 SDK 时仍可编译 UVC 和视频回放入口，选用未编译的 SDK 相机会明确报错。也可在正常工程构建完成配置后执行 `cmake --build build --target calibration_capture`，生成 `bin/calibration_capture`。

## 1. 准备标定板与相机

编辑 `config/calibration.yaml`：默认是 **10 列 × 7 行对称圆点板、相邻圆心 40 mm**。数字指圆心数量，不是格子数量。拍照时预览必须识别出全部圆点。与实际部署保持相同的分辨率、ROI、翻转、镜头焦距及对焦位置；关闭自动曝光，适当提高曝光以看清标定板。

拍照程序直接读取机器人配置的 `camera`（默认 `config/sentry.yaml`），不用复制海康参数到同济格式。不要与 `sentry`/`standard` 同时占用相机和串口。输出目录必须尚不存在，程序会保存配置快照，防止混合不同采集参数。

## 2. 拍照

```bash
# 只标定相机内参，不需要下位机或 IMU。
build/calibration/calibration_capture config/sentry.yaml \
  --intrinsics-only=true --output=output/calibration/intrinsics-01

# 外参采集：同步保存 PNG 原图及同名 TXT 四元数（w x y z）。
build/calibration/calibration_capture config/sentry.yaml \
  --output=output/calibration/handeye-01
```

按 `s` 保存，`q`/Esc 退出，Ctrl+C 也可结束。预览缩放及标记只用于显示，保存图像为原分辨率、无标记的 PNG。内参可移动标定板，采集至少 10 张（建议 20–30 张）覆盖画面中心、边缘、距离和倾角的照片。

外参采集时**标定板固定、云台旋转中心位置固定**，分别改变 yaw 和 pitch，并停稳后保存；只转一个轴无法解算平移。保持板面朝向和圆点编号一致，避免对称板 180° 翻转。相机镜头、板或底盘移动后须重新采集。`pose ready` 才能保存外参样本；姿态缺失、过期、运动中或圆点不完整均拒绝保存，不填造单位四元数。

外参使用现有串口状态 ID（配置 `serial.telemetry_gimbal_id` 和 `serial.telemetry_shoot_id`），将固件返回的度数按 `Rz(yaw) Ry(pitch) Rx(roll)` 转成四元数，与 `sentry.cpp` 一致。没有设备时间戳，使用接收处理时间及 `serial_send_to_image_microseconds` 补偿，因此要求静止采集；不是运动中硬件同步。该工具只接收姿态，不发送运动/射击指令。

实时采集的四元数已经是 gimbal→world，`R_gimbal2imubody` 必须为单位阵。导入同济原始 JPG/TXT 数据时，才将其设置为实际 IMU 安装旋转；离线工具会按同济的 `Mᵀ Q M` 转换。支持 JPG/PNG/BMP、文件编号不连续；重复同名不同扩展名、损坏姿态、混合分辨率会报错。

可用 `--camera-key=armor_omni.camera0 --intrinsics-only=true` 标定其他相机内参。外参输出只支持主相机。无显示环境可添加 `--headless=true --interval=2 --count=20` 自动采集，需自行改变标定姿态；重复静止照片不能替代多姿态数据。自动模式仍执行圆点和姿态检查，Ctrl+C 可终止等待。

## 3. 内参标定与输出

```bash
python calibration/calibrate.py camera output/calibration/intrinsics-01 \
  --robot-config config/sentry.yaml --output output/calibration/sentry-intrinsics.yaml
```

默认无 GUI 批处理，添加 `--preview` 可逐张检查圆点。使用同济的 `CALIB_FIX_K3` 和 100 次迭代，输出逐张重投影 RMS 与整体 RMS；默认超过 2 px 拒绝生成结果。质量阈值可在 `config/calibration.yaml` 调整。应查看逐张误差并重拍质量差的视角，不应仅放宽阈值。

输出是**完整的新机器人 YAML**，可供下一步或机器人程序直接读取。只替换指定相机的 `camera_info`，其他相机参数、串口及算法配置数值保持原样；YAML 注释与排版不保留。原文件不修改，已存在的输出文件也不覆盖。

| 武科字段 | 输出内容 |
| --- | --- |
| `image_width` / `image_height` | 原始标定图片尺寸 |
| `camera_matrix` | `rows: 3, cols: 3, data: [...]`，行优先 |
| `distortion_model` | `plumb_bob` |
| `distortion_coefficients` | `rows: 1, cols: 5`，k1,k2,p1,p2,k3（k3=0） |
| `rectification_matrix` | 3×3 单位阵，单目校正旋转 |
| `projection_matrix` | 3×4 `[K | 0]`，无双目基线 |

## 4. 外参标定与输出

```bash
# 读取刚生成的内参配置，解算 Tsai 手眼。
python calibration/calibrate.py handeye output/calibration/handeye-01 \
  --robot-config output/calibration/sentry-intrinsics.yaml \
  --output output/calibration/sentry-calibrated.yaml

# 或用同济的 Shah 机器人世界／手眼联合解算作对比。
python calibration/calibrate.py robotworld-handeye output/calibration/handeye-01 \
  --robot-config output/calibration/sentry-intrinsics.yaml \
  --output output/calibration/sentry-robotworld.yaml
```

两种方式均使用平面 IPPE PnP、固定云台原点（平移为零）的假设，并拒绝姿态激励不足的数据。标定后检查固定板在世界坐标系的位置/旋转一致性，默认平移 RMS ≤ 0.02 m、旋转 RMS ≤ 3°。失败时不输出可部署配置。联合标定额外报告板的世界位置，沿用同济板坐标原点约定。

写入武科 `tf.camera_in_gimbal`：

```text
R_wust = R_camera_cv_to_gimbal × transpose(R_CV2PHYSICS)
t_wust = t_camera_cv_to_gimbal_mm / 1000
```

`R` 是 9 个行优先数，`t` 是 3 个以米为单位的数。运行时武科会再乘 `R_CV2PHYSICS`，所以这里必须扣除，不能直接复制同济 `R_camera2gimbal`。不会改写 `shoot_in_gimbal` 等无法由本标定测出的机械外参。最终配置数值需在实机上核验，合成测试不能代替真实采集。

## 配套录像裁剪与测试

```bash
# 输入为 recording.avi 与 recording.txt，每行 t w x y z。
# 零起始帧号，包含 end 帧；输出新文件段，不覆盖输入。
python calibration/split_video.py records/recording output/calibration/clip --start 100 --end 300

# 无硬件回归测试：内参恢复、两种外参、坐标转换、图片检测及异常输入。
python -m unittest discover -s tests -p test_calibration.py -v

# Linux：运行实际拍照二进制，使用视频和伪终端模拟相机／串口。
python tests/run_capture_tests.py build/calibration/calibration_capture
```
