# 真机运行指南

适用环境：课程工作站、UR7e、腕部 D435i、ROS 2 Humble。以下按 2026-11-24 最后一次成功的小脸试画整理。每次运动由现场操作者手动启动，另一人负责急停。

## 1. 已验证方案与夹笔基准

**今后统一夹法：笔尾（远离笔尖的一端）直接抵住法兰盘，再由夹爪夹紧。** 保持笔轴沿腕部 +Z，检查笔尖横向偏移和夹持牢固程度。不要把“抵住法兰盘”理解为笔尖接触法兰盘。

这个机械止挡用于重复定位。同一支笔、同一支架和夹爪安装可沿用已核验参数；换笔、磨损、笔尖伸缩、偏心或支架移动后仍须复核。不能仅凭抵住法兰盘就假定所有笔的 TCP 相同。

本次工位成功配置：

| 项目 | 数值/行为 |
| --- | --- |
| 有效 TCP | `tip_in_link: [0.0, 0.0, 0.259]` m，以 `wrist_3_link` 表达 |
| 落笔 | `--hover 0`，沿标定纸面定位，无负下压 |
| 滑块 | 80%，不是机器人额定最大速度的 80% |
| 原轨迹限速 | 关节 0.10 rad/s、加速度 0.20 rad/s²、笔尖 0.015 m/s；滑块再缩放 |
| 抬笔 | 15 mm |
| 起点允许距离 | `motion.max_approach: 0.20` m |
| 计时 | `motion.geometry_tip_timing: true`，双 URDF 保守速度界 |
| 每次执行 | 从实时状态重新完整规划、检查碰撞；不重放旧轨迹 |
| 起止流程 | 先安全返回保存起点 → 画小脸 → 在末笔上方抬笔停止 |

259 mm 是本工位的**有效参数**：间隙核验得到约 264 mm，最后总修正 5 mm 后成功接触试画；不是在 259 mm 基础上继续向下压 5 mm，也不是独立 pivot TCP 标定结果。后续若仍有间隙，应检查笔、相机和纸面，不持续盲目增加补偿。没有笔压闭环，不允许负 hover 硬压桌面。

最终本地记录 `runs/smiley-contact-20261124-140300/plan.json` 为 `executed: true`，223 点、名义轨迹约 262.6 秒；80% 滑块下绘画约 5 分 28 秒，加返回起点通常合计约 6 分钟。操作者确认执行成功。此结果仅代表这支笔和本工位，不代表所有安装均已验证。

## 2. 容器、安装与更新

宿主机的新终端先进入容器：

```bash
distrobox enter ros2
```

已在容器、能运行 `ros2` 时，不要再次调用 `distrobox`。各容器终端均进入项目并加载环境：

```bash
cd ~/ros_workspaces/Doodle_Ur7e
source /opt/ros/humble/setup.bash
source install/setup.bash
```

保留课程分配的 `ROS_DOMAIN_ID`、`EE106_UR_ROBOT_NAME`、`EE106_UR_ROBOT_IP`。不从其他工位复制，也不把隔离测试域 231 用于真机。

首次部署才需要：

```bash
cd ~/ros_workspaces
git clone https://github.com/HARROLDX/Doodle_Ur7e.git
cd Doodle_Ur7e
sudo apt-get update
sudo apt-get install python3-opencv python3-numpy python3-scipy python3-yaml python3-pil ros-humble-cv-bridge
bash scripts/build_lab.sh
source install/setup.bash
test -f config/local.yaml || cp config/lab.yaml config/local.yaml
```

课程镜像还须有 `ur7e_utils`、支持 ur7e 的 `ur_description`/`ur_moveit_config`、`realsense2_camera`、`ros2bag`。私人仓库需要自己的访问权限。模板值和 `verified: false` 不可当作现场事实；按[标定说明](docs/calibration.md)配置。

已有副本先保留自己的修改，适合快进更新时使用 `git pull --ff-only`，随后构建。不要覆盖 `config/local.yaml`。

## 3. 启动唯一的驱动、相机与 RViz

接好 D435i USB 3 数据线；只保留一套驱动、一个相机节点和一个 RViz。已有相应进程时不要重复启动。

终端 A：**夹笔之前**启动并保持运行：

```bash
ros2 run ur7e_utils enable_comms
```

这个命令初始化时会开合夹爪。已经持笔时先安全取下笔，不为普通 ROS 超时持笔重启通信。

终端 B：

```bash
ros2 launch doodle_ur7e bringup.launch.py config:="$PWD/config/local.yaml"
```

它启动相机、MoveIt/RViz、腕部相机 TF，不发送绘画轨迹。已有相机用 `camera:=false`；已有 MoveIt/RViz 用 `moveit:=false`；已有正确唯一的安装 TF 用 `mount:=false`。不要叠加 Lab5/Lab7 的相机 TF、抓取或视觉伺服节点。

如果启动时没接相机，连接后相机可恢复，但 mount 可能已经超时退出；确认没有同类 TF 发布者后，仅补启：

```bash
ros2 run doodle_ur7e doodle --config config/local.yaml mount
```

RViz 中添加 `Image`，topic 选 `/camera/camera/color/image_raw`，Reliability 选 `Best Effort`，即可看 RealSense 彩色视角。保留现有窗口，不重复打开相机。

## 4. 夹笔、Freedrive 与恢复

终端 C：以下同一命令交替开/合，每次只调用一次并观察：

```bash
ros2 service call /toggle_gripper std_srvs/srv/Trigger '{}'
```

确认夹爪打开 → 笔尾直接抵住法兰盘并对齐 → 闭合夹爪 → 检查笔不滑动、手已离开。随后手动定位：

```bash
ros2 run ur7e_utils freedrive
```

调整到笔尖向下、绘图区附近、相机能看见四码的高位观察姿态。新保存起点建议至少 115 mm 净空，便于先做 100 mm 悬空验收；当前已保存并核验的工位起点无需重录。

按 Ctrl+C 退出 Freedrive，再显式恢复轨迹控制器：

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  '{activate_controllers: [scaled_joint_trajectory_controller], deactivate_controllers: [freedrive_mode_controller, forward_position_controller, forward_velocity_controller], strictness: 1, activate_asap: true, timeout: {sec: 5, nanosec: 0}}'
ros2 control list_controllers
```

必须返回 `ok: true`，scaled 控制器为 `active`，Freedrive、forward position/velocity 均非 active。运行脚本不自动切换控制器、不自动夹笔、不 tuck。

## 5. 四码、纸面标定与固定起点

打印 [A4 一体式纸板](docs/print/ur7e_drawing_board_A4.pdf)，实际尺寸 100%；核对黑色码边长 25 mm、中心距 150 mm。使用 `DICT_4X4_50` 的 ID 0–3，不是 Lab7 的 5X5 字典。

```bash
python3 scripts/check_markers.py
```

默认读取 90 帧新鲜唯一图像，输出每个 ID 的检出次数和“四码同时出现”帧数。没有图像查 USB，少码先调视野。

初始化或配置变化后：

```bash
ros2 run doodle_ur7e doodle --config config/local.yaml scene
ros2 run doodle_ur7e doodle --config config/local.yaml doctor
```

`scene` 更新本项目碰撞盒；`doctor` 检查静止状态、姿态、控制器、场景、图像和曝光时刻 TF。二者不发运动。桌面、笔身、相机支架等包络必须与实物相符，不删除障碍物或关闭碰撞来让规划通过。

**纸张移动就重新标定。** 先保存为新文件，不覆盖仍在使用的已核验纸面：

```bash
ros2 run doodle_ur7e doodle --config config/local.yaml calibrate --output runs/paper-new.yaml
```

标定用 12 帧求纸面，输出默认 `verified: false`。现场核对 XY、方向和纸高后记录核验方法，再设为 true；相机检测通过不等于高度已实测。使用下文的 `PAPER_FILE` 指定新文件。当前已成功工位的标准文件 `runs/paper.yaml` 已整理为最后成功时使用的标定，旧文件另存备份。

准备小脸笔画，不移动机器人：

```bash
ros2 run doodle_ur7e doodle prepare images/external/smiley.png --output runs/smiley
```

首次建立工位时，处于上述高位并完成配置/纸面核验后保存起点：

```bash
python3 scripts/save_drawing_start.py --paper runs/paper.yaml
```

这只保存静止姿态，不移动；默认输出 `runs/drawing_start.json`，已存在时拒绝覆盖。不要把绘画末端姿态覆盖到已验收起点。确需新起点时使用不同 `--output`，并重新悬空验收。

当前日常流程需要的文件：

| 用途 | 默认文件 |
| --- | --- |
| 现场参数 | `config/local.yaml` |
| 已核验纸面 | `runs/paper.yaml` |
| 保存的高位起点 | `runs/drawing_start.json` |
| 小脸笔画和预览 | `runs/smiley/strokes.json`、`preview.svg` |

现场参数、起点、标定和录像均不上传 GitHub；新克隆必须在工位重新创建，不能用示例或仿真数据代替。当前起点仍保留上次成功试画的初始关节姿态，不因文件整理而改变。

## 6. 每次从固定起点画小脸

完成上述准备后，在容器项目目录运行：

```bash
bash scripts/run_smiley_from_start.sh
```

脚本依次检查本地输入文件、设置并确认滑块 80%、从实时状态完整规划并返回保存起点、再实时规划和执行 `hover=0` 小脸。任一步失败即停止。先抬到两端较高净空再加 15 mm，再平移回起点；并非直接插值关节或重放保存轨迹。画完在末笔上方抬笔 15 mm 停止。

要先只看返回路径：

```bash
python3 scripts/return_drawing_start.py
```

只返回，不画图：

```bash
python3 scripts/return_drawing_start.py --execute --supervised
```

单独规划接触小脸，不执行：

```bash
ros2 run doodle_ur7e doodle --config config/local.yaml draw \
  runs/smiley/strokes.json --paper runs/paper.yaml \
  --hover 0 --output runs/smiley-contact-plan
```

使用新标定/新起点：

```bash
PAPER_FILE=runs/paper-new.yaml START_FILE=runs/start-new.json \
  bash scripts/run_smiley_from_start.sh
```

还可通过 `CONFIG_FILE`、`STROKES_FILE` 指定其他本地配置/笔画。返回脚本也有 `--config`、`--paper`、`--start` 参数；新图案须重新检查规划。

新笔或新工位先做 100 mm → 50 mm → 按需更低正高度 → 接触验收；使用 CLI 的 `--hover 0.10`、`0.05`、`0`，每一级先规划、再由现场验收。CLI 默认仍是 100 mm，当前脚本显式传 0。不要对新工位直接使用本次接触参数。

输出在 `runs/smiley-contact-日期-时间/`。只有 controller action 成功终止才写 `plan.json` 的 `executed: true`；取消不能当作成功。

## 7. 记录与结束

如需调参记录，先在独立容器终端启动：

```bash
python3 scripts/record_trial.py --output "runs/record-$(date +%Y%m%d-%H%M%S)" --duration 1200
```

它只记录 RealSense MP4、逐帧 ROS/接收时间戳、关节/控制器/TF/力反馈 rosbag 和输入快照，不发送运动。默认 20 分钟自动结束；看见 `VIDEO_RECORDING_READY` 并核对 rosbag 日志后，再在另一终端执行画图。视频标称 10 fps，实际时间对齐以 CSV 为准。Ctrl+C 停止此记录器不会取消其他终端的绘画。

停止绘画时，在**绘画终端**按 Ctrl+C，等待取消终态并确认机器人停稳。不用 Ctrl+Z，不叠加运动命令；终态未确认、明显顶纸/弯笔或状态不明时立即物理急停。

结束后先安全离桌、取下笔，确认归位路径无遮挡，再手动：

```bash
ros2 run ur7e_utils tuck
```

急停后先排除原因、按课程流程释放急停并保持原通信运行，再由现场操作者运行 `ros2 run ur7e_utils reset_state`。不要持笔重启会开合夹爪的 `enable_comms`。

## 8. 常见问题与开发验证

- `distrobox: command not found` 且能运行 ros2：通常已在容器内，不要重复进入。
- `complete measured config verification`：配置或纸面未核验；检查实物，不直接翻验证开关。
- 只读控制器/场景查询超时：有限重试一次，两次失败即停止。运动 action 不自动重试；不为此重启通信。
- `robot must be stationary`：先确认正在执行的任务并等待/正确取消，再重试。
- 取消后低位返回：计算净空至少 1 mm，首段只上抬；两套 URDF 校验插值全过程保持正净空。已接触/压住纸面或净空不明时，应由现场操作者处理，不能强行返回。
- `incomplete path`、起点过远或姿态不符：调整姿态/环境并重规划，不接受部分路径。
- 相机/TF 时间戳异常：检查唯一相机和安装 TF；不改用最新 TF 代替曝光时刻 TF。

图片/GPT入口见 [Baymax](docs/baymax.md)，测量见 [标定说明](docs/calibration.md)，尺寸见 [打印说明](docs/print/README.md)，来源见 [third_party](third_party/README.md)。

开发测试在容器运行 `bash scripts/test_humble.sh`，其 localhost/domain231 与真机隔离。实际模型离线检查为 `ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=231 python3 scripts/verify_moveit.py`，不连接驱动或发送 action。测试和规划通过不替代现场验收。
