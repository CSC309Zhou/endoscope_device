# endoscope_device

ROS2 Humble 内窥镜节点，默认同机传输、1920×1080、MJPG，目标帧率20fps。

## 编译与启动

将整个软件包放到工作空间的 `src/endoscope_device/` 下。以下用 `~/ros2_ws` 表示你的工作空间，请替换为实际路径：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select endoscope_device --symlink-install
source install/setup.bash
ros2 launch endoscope_device endoscope.launch.py
```

启动前关闭VLC等占用摄像头的程序；按 `Ctrl+C` 停止节点。

## 修改相机参数

编辑 **`src/endoscope_device/config/endoscope.yaml`**。每项注释说明了用途、范围和生效条件。

启动命令默认加载包内的 `config/endoscope.yaml`。按上面的 `--symlink-install` 方式编译后，安装配置链接到源码文件：**只修改配置，保存后重启节点即可，不需要重新编译。** 修改C++源码后才需要重新执行编译命令。只有使用其他参数文件时，才需要添加 `config:=/绝对路径/其他配置.yaml`。 如果之前没有使用 `--symlink-install`，安装目录是普通副本，修改源码配置后需要重新编译同步。

例如，手动10毫秒曝光修改这两项：

```yaml
controls.auto_exposure: false
controls.exposure_time_us: 10000
```

修改手动色温前，将 `controls.auto_white_balance` 设为 `false`。这些设备控制都要求 `controls.apply: true`；设为 `false` 时保留设备原有控制状态。

## 发布的话题与图像查看

| 话题 | 消息类型 | 内容 |
|---|---|---|
| `/endoscope/image_raw` | `sensor_msgs/msg/Image` | bgr8彩色图像 |
| `/endoscope/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG图像 |
| `/endoscope/camera_info` | `sensor_msgs/msg/CameraInfo` | 分辨率、标定信息；无标定时K为0 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 实际采集帧率、设备状态等 |

新开终端，先加载同一个工作空间，再打开实时图像：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /endoscope/image_raw
```

## 查看当前参数和实际帧率

以下命令也在已加载工作空间的终端执行：

```bash
# 一次查看所有当前设置：分辨率、目标帧率、曝光、白平衡等
ros2 param dump /endoscope

# 也可以单独查询；参数名与YAML中的名称一致
ros2 param get /endoscope controls.exposure_time_us

# 查看实际接收到的图像帧率（不是配置中的目标帧率）
ros2 topic hz /endoscope/image_raw/compressed

# 查看设备状态；capture_fps为实际采集帧率
ros2 topic echo /diagnostics --once
```

`width`、`height` 是分辨率，`fps` 是目标帧率。自动曝光开启时，`controls.exposure_time_us` 只是手动备用值，不代表实时曝光时间；自动白平衡开启时，手动色温也只是备用值。诊断中的曝光读回值也可能是驱动缓存。

需要临时调整时，例如：

```bash
ros2 param set /endoscope controls.auto_exposure false
ros2 param set /endoscope controls.exposure_time_us 10000
```

临时调整不会写回YAML，重启后重新读取文件。分辨率、格式等采集设置请改文件后重启。

## 本机传输

默认使用Fast DDS和 `config/fastdds_local.xml` 中的64MiB共享内存配置。**发布端和查看端都先source同一工作空间**，即可自动加载，无需手动export；该配置用于同机通信。 如果之前在其他环境运行过ROS2命令，旧发现服务可能保留旧配置；在加载本工作空间后执行一次 `ros2 daemon stop`，再运行 `ros2 topic list` 即可重新发现。

此前实测：当前光照下自动曝光约16.7fps，手动10毫秒约19.5fps。共享内存解决大图传输掉帧，自动曝光仍可能影响设备实际采集帧率。
