# 现场标定与故障定位

本文件不包含自动真机运动。所有手动运动遵循课程规程，由现场人员监督。

## 坐标与笔尖

所有长度 m，角度 rad，四元数 xyzw。`base_link` 不是原版 Franka 的 `base`。`tip_in_link` 是腕部坐标中从 `wrist_3_link` 原点指向笔尖的完整向量。目标腕部位置为

`p_base_wrist = p_base_tip - R_base_wrist * tip_in_link`。

Lab 5 的抓取长度不是笔尖 TCP；测量时把夹爪、笔伸出长度和横向偏心全部计入。在多个受监督姿态中用已知物理点检查笔尖的计算位置，应在所需绘画精度内一致。更高精度可用固定笔尖 pivot calibration 或课程认可的 TCP 标定工具，不能仅用单张照片推断毫米级参数。每次换笔或重夹要复核。建议有小行程弹性笔夹／软笔尖，初次接触从零下压量开始。

模板 `[0,1,0,0]` 来自 Lab 5 朝下姿态；先用 RViz 与 TF 核对。程序要求腕部 +Z 朝向纸面，起始姿态与配置差小于 0.03rad。若实际夹笔轴不同，需要调整夹持方式及姿态，不能单独把验证开关置 true。

2026-11-24 工位采用笔尾（远离笔尖的一端）直接抵住法兰盘后再夹紧的重复定位方式。当前同一支笔/安装的小脸试验使用有效TCP `[0,0,0.259]`m并由操作者确认执行成功；它包含本轮间隙校准误差，不是所有笔通用的机械长度。换笔、笔尖磨损/伸缩、横向偏心或安装变化后仍须复核，不直接沿用。具体流程见 [START](../START.md)。

## 相机 TF

Lab5 将 `[-0.025,0.13,0]`、单位旋转赋给深度光学帧，Lab7 将同一数值赋给彩色光学帧。这两种定义不能混为同一份精确标定。新模板采用 Lab7 的 `camera_color_optical_frame`，与彩色 ArUco 图像一致；现场仍需验证。已有 local.yaml 不会自动更改：保留已测量的外参及其对应 optical_frame，不要只改帧名而沿用另一帧的测量值。

RealSense 本身发布 `camera_link → … → optical_frame`。`mount` 使用配置选定的光学帧计算

`T_wrist_camera_link = T_wrist_optical * inverse(T_camera_link_optical)`

只发布腕部到 camera_link 的静态连接，等待 RealSense 内部 TF 最长60秒。不要同时运行 Lab5 `planning tf` 或 Lab7 `visual_servoing tf` 再给 optical frame 发布第二个父节点。彩色定位采用图像消息真实光学帧和曝光时刻 TF。若 camera_name 或 namespace 改过，修改配置中的 topic/frame 名称。

`ros2 run tf2_tools view_frames` 查看唯一且连通的 TF 树。在至少两种腕部姿态下观察固定桌面上的标记，转换到 base_link 后应保持不动；若明显漂移，应修正安装外参、时间同步或 TF 连接。网络跨机时还要保证时间同步。`calibrate` 只查询曝光时刻 TF，不退回 latest。

## 标记板与纸面

生成的 board.svg 使用 DICT_4X4_50，既不是 Lab7 的 DICT_5X5_250（5/15cm）标记，也不是原项目的 AprilTag PDF。默认 150×150mm 是标记**中心间距**，marker 黑色外边长 25mm。ID 0 左下、1 右下、2 右上、3 左上；纸面原点是 0 的中心，+X 向右、+Y 向上、+Z 离开桌面。绘画范围留 15mm margin，默认中心 120×120mm。改变尺寸后重新生成、打印和标定。

SVG 物理尺寸约 199×199mm，可用 A4 以实际尺寸打印。四角标记与绘图区共面，禁止拉伸打印。`calibrate` 要求四个标记同时出现，并用实际 CameraInfo 内参计算 PnP；12 帧位置散布小于 2mm、姿态散布小于 0.02rad、单帧重投影 RMS 小于 1.5px。这只是稳定性检查，不能证明绝对接触高度准确。

输出 `runs/paper.yaml` 的 `verified` 默认为 false。先在 RViz 和悬空检查 XY 朝向及位置，再以课程认可的触点／测量方式核实纸面高度，必要时修正 position 沿纸面法向的分量。不要靠图像估计直接压笔到桌面。记录所用测量方法、纸层高度差，再设 `verified: true`。整个桌面倾斜应小于 10°。纸移动即重新标定。

## 碰撞和起点

配置的 world_boxes 和 attached_boxes 来自 Lab 5 尺寸示例，不能假定每个工位相同。保留桌面碰撞，不要为了规划成功把桌子移走。笔身碰撞盒应覆盖真实刚体并在笔尖前略微结束，模板留 5mm。允许 touch_links 只针对真实相连的机器人 link，不给整张桌子设置碰撞豁免。核对支架与夹爪碰撞盒，过宽会导致模型自碰撞，过小会漏检。

本项目场景使用 `doodle_` 前缀，不删除 Lab 5 或其他程序留下的场景对象。若重复对象导致碰撞，先在 RViz 明确归属，再由操作者管理；不要启动完整 Lab5 bringup 自动叠加旧 TF 和抓取逻辑。

起点必须已在纸面上方，朝向与配置一致。默认 `hover=0.10` 且抬笔 `clearance=0.015`，因此起始笔尖至少高于纸面约 115mm。只会在已确认高处平移，再下降到首点。任一规划失败时检查姿态、碰撞盒、TCP 和可达性，不能关闭碰撞检查或接受部分路径。

## 常见失败

- `ur7e` 不被 launch 支持：课程容器的 UR 包版本不匹配，联系课程环境维护者；不要伪装成 UR5e。
- FK、MoveIt service 不存在：确认进入 ros2 容器且与机器人同 ROS_DOMAIN_ID，MoveIt 已启动。
- `scaled controller inactive`：自由拖动尚未结束或课程 controller 没恢复；按 START 的 Freedrive 与恢复一节执行显式 SwitchController 命令并核查 active 状态，本程序不会自动切换。
- `scene geometry differs`：修改配置后重新运行 scene 并检查 RViz；不要仅翻 verified 标志。
- `need exactly board markers`：改善视角/光照、确保整块板可见且尺寸正确，不接受旧相机帧。
- `joint state timestamp stale`／TF extrapolation：检查 ROS 时间、消息流、跨机时间同步；程序拒绝 latest 兜底。
- `incomplete path`：先用方框检查可达区、模型碰撞、腕部姿态；不执行 partial trajectory。
