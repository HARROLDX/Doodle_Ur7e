# Lab5 / Lab7 真机接口对照审查（2026-10-28）

参考课程仓库 `ccdfc55`（Complete Lab 7 visual servoing）；项目审查基线 `6411a61`。用户确认课程实验已真机验证；本次读取代码和指南作为接口依据，没有操作真实机械臂。

| 项目 | 课程证据 | 审查结果 |
|---|---|---|
| 环境 | lab5/README.md：Distrobox Humble、课程工作站环境变量 | 一致，继续使用系统Python和课程ROS域；服务器不连接真机 |
| MoveIt | lab5 planning/main.py、ik.py和bringup | group ur_manipulator、base_link、wrist_3_link、UR7e和scaled action一致 |
| 控制器切换 | Lab5 SwitchController；Lab7 main.py启用forward_velocity_controller | 项目执行前检查scaled active并拒绝冲突控制器；启动指南明确停止Lab7伺服后切回 |
| 图像 | lab7 ros2_aruco/config/aruco_parameters.yaml | color/image_raw、color/camera_info及sensor QoS一致；本项目按曝光时间查TF |
| 安装外参 | Lab5 static_tf_transform.py为depth optical；Lab7为color optical | 同数值不等于同一精确标定。新模板采用Lab7彩色定义；旧实测配置不自动迁移 |
| TF拓扑 | RealSense内部TF＋课程腕部连接 | 保留项目腕部到camera_link的组合变换，避免重复父节点；新增非单位旋转测试验证矩阵方向 |
| ArUco | Lab7 DICT_5X5_250，按ID采用0.05/0.15m | 与项目DICT_4X4_50、ID0–3、0.025m不同，是有意设计；必须打印项目board，不能直接用Lab7标签 |
| 笔尖 | Lab5 GRIPPER_LENGTH=0.14m用于抓取 | 不迁移到笔尖TCP；当前0.22m仍是模板，须测完整XYZ |
| 运动方式 | Lab5离散抓放；Lab7视觉速度伺服 | 项目保持固定纸面标定＋完整路径规划，未混入持续速度控制，不需Lab7 marker/goal_pose节点 |

## 本轮调整

没有发现需要重写主要控制链的接口冲突。以下改动针对可复现的部署差异/操作易错点：

- lab.yaml默认参考光学帧改为Lab7彩色帧；保留安装外参现场核验。旧local.yaml和纸面标定不会被覆盖，不应盲改旧实测外参的帧名。
- mount等待内部TF由10秒放宽到有界60秒，避免相机冷启动稍慢时提前退出；不回退到猜测外参。
- doctor显示当前笔尖XYZ及距配置姿态的角度差，解决“能看见码但不满足0.03rad起始姿态限制”难定位的问题；不自动调整机器人姿态。
- 新增START.md，整合启动顺序和两种课程控制器之间的切换，明确禁止重复TF/相机/MoveIt、标记板差异和首次悬空验收。
- Baymax现场构建命令只选择doodle_ur7e，不要求编译Isaac专用硬件插件。

## 验证结果

A6000上既有Humble容器，`--network none`、ROS_DOMAIN_ID=231、无GPU、未启动硬件驱动：

- 40项回归通过，包含相机原生非单位旋转与平移下的安装变换组合测试，确认只发布wrist→camera_link。
- 实际UR7e MoveIt：悬空路径91点、接触路径93点，均完整通过；故意加入障碍后路径fraction=0并拒绝。
- 日志：服务器 `/home/msc-auto/Harroldx/Doodle_Ur7e/runs/lab57_review_20261028.log`。

## 仍须现场确认

代码和启动接口已与课程对齐，不能据此断言本项目真机零问题。剩余关键项是实际笔尖TCP、安装外参、纸面接触高度、工位碰撞尺寸和初始姿态；先通过小方框悬空，再接触纸面。绘画无笔压闭环，纸张须固定，轨迹采用保守逐点停顿时间设置。既有完整Isaac验证保留，本轮不重复无关GPU渲染。

参考：[Lab5指南](https://github.com/HARROLDX/eecs106a-fa26/blob/ccdfc55/lab5/README.md)、[Lab7安装TF](https://github.com/HARROLDX/eecs106a-fa26/blob/ccdfc55/lab7/src/visual_servoing/visual_servoing/static_tf_transform.py)、[Lab7相机/标记配置](https://github.com/HARROLDX/eecs106a-fa26/blob/ccdfc55/lab7/src/ros2_aruco/config/aruco_parameters.yaml)、[Lab7控制入口](https://github.com/HARROLDX/eecs106a-fa26/blob/ccdfc55/lab7/src/visual_servoing/visual_servoing/main.py)。
