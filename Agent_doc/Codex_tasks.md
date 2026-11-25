# 项目入口与当前任务

## 2026-11-24：最终成功与文档/GitHub整理

用户确认总补偿5mm的小脸完整执行成功；本地runs/smiley-contact-20261124-140300/plan.json证实executed=true、223点、名义时长262.57s，使用有效TCP259mm、hover=0、滑块80%。用户规定以后笔尾直接抵住法兰盘再夹紧，作为重复定位基准；换笔/磨损/安装变化仍需复核。此次新请求仅为整理并上传GitHub，不额外启动真机动作。

README/START按成功流程重写，保留仿真视频和归属；新增可重复的保存起点与被动记录入口。脚本默认统一config/local.yaml、runs/paper.yaml、runs/drawing_start.json、runs/smiley/strokes.json，支持指定其他文件；当前工位只迁移纸面文件别名和重新生成同一份笔画，不改变已保存起点。旧runs/paper.yaml备份在runs/paper-before-successful-trial.yaml，原历史标定仍保留；新笔画与现场版本完全一致。缺本地文件时在改速度/运动前停止，确认滑块成功后才返回与画图。save_drawing_start.py只保存高位静止姿态并拒绝覆盖既有起点，当前已保存起点不重录。

最终59项Humble测试通过，官方UR7e MoveIt离线悬空/接触规划和障碍拒绝通过。记录工具4秒28帧实测，视频/时间戳/rosbag正常。收尾只读检查时当前腕部姿态已与保存起点不一致，返回工具正确拒绝；未移动机器人或覆盖起点。只提交代码、文档、测试，不提交config/local.yaml、runs/或用户.vscode设置；模板verified仍false。用户已明确授权整理后提交并推送HARROLDX/Doodle_Ur7e。

## 2026-11-24：总补偿由3mm调至5mm

用户反馈3mm补偿不足，要求总补偿5mm；确认机械臂静止后，将有效TCP从261mm再减2mm至259mm，笔身包络中心z改为.1815m（末端距tip仍留5mm），同步START。不是在261mm基础上再减5mm。保持hover=0、滑块80%、固定起点和每次实时规划；新补偿尚未执行验收。

## 2026-11-24：3mm残余间隙修正与START重写

操作者观察hover=0试画仍离纸约3mm，要求补偿并交付命令；有效TCP从264mm减至261mm（同纸面目标下腕部向纸面移动约3mm），笔身包络中心z由.1865改为.1835，仍保留hover=0、不允许负下压。此次新补偿尚未实际执行，不能宣称已接触绘画成功。调整说明存runs/smiley-20261124-133152/tcp_adjustment_3mm.json。

当前机械臂已取消并停在低位，261mm模型计算净空约3.01mm。返回脚本允许至少1mm正净空的低位起点，首段先抬高；两套URDF逐段采样并加二阶导数余项核验始终保留至少1mm纸面净空，仍检查完整路径/碰撞/关节跳变及执行前静止状态。现场仅规划通过：返回14点30.1s@100%；假定保存起点的接触路径223点262.4s@100%（明确离线假定起点，不发送动作），80%预计返回+画图约6分钟。/get_planning_scene只读查询也增加一次有限重试；场景写入、规划及运动action均不自动重试。48项Humble回归通过。

START.md按当前现场流程重写，标明261mm为待落笔验收的有效TCP修正、忽略的本地产物依赖、容器内无需再distrobox、唯一相机/RViz、夹爪/Freedrive/恢复、纸张移动即重标定、固定起点返回、80%接触脚本、记录时限及取消语义。新增scripts/check_markers.py复用项目DICT_4X4_50检测90帧新鲜唯一图像，供以后重复检查。

## 2026-11-24：现场TCP确认、固定起点与接触绘画入口

现场返回起点前曾在/controller_manager/list_controllers响应等待超时，尚未发送返回动作；复查服务约1s响应、控制器active且安全状态NORMAL。增加仅该只读查询超时后一次有限重试并清理pending future；不重试规划写操作/场景写操作/运动action，不改变超时后的停止原则。新增故障注入测试覆盖可恢复、两次失败及非只读请求不重试，47项Humble回归通过。用户随后重启脚本，返回action成功，接触小脸进入执行；最终执行结果须查对应runs/smiley-contact-*/plan.json，勿把执行中视为成功。

用户实测上抬试验后的垂直间隙105mm，并明确接受由单姿态反推的有效TCP264mm；local.yaml的pen_tcp=true是用户现场确认，不代表独立pivot标定。新纸面runs/trial-20261124-131854/paper_for_smiley.yaml已12帧标定，当前264mm配置计算间隙105.014mm，与用户读数一致，记录验证方法后verified=true。原始测量与限制保留在runs/trial-20261124-131854/manual_clearance_check.json。笔身包络同步更新。

上次50mm悬空小脸被用户取消（action状态5，executed=false），未宣称完整真机画完。用户要求保存上次执行起点并改为80%滑块、hover=0接触。起点存runs/drawing_start.json，来自该次execution/plan.json首关节状态（上抬10mm之后的位置，并非更早的Freedrive姿态）。scripts/return_drawing_start.py每次从实时状态重新规划，先抬至两个端点较高纸面净空再加15mm，平移后回到所存姿态；保留控制器、碰撞、完整率、跳变、双URDF速度界及执行前状态检查。不tuck，不复放保存轨迹。现场仅规划：返回12点21.7s@100%，接触小脸218点245.7s@100%。scripts/run_smiley_from_start.sh由用户手动执行，设置80%后返回，成功终止后才重新规划并执行hover=0。无负hover、无笔压闭环，未声称接触测试通过。

## 2026-11-24：现场计时过慢修复

用户要求降低小脸悬空试验耗时并保持真机安全，目标hover=0.05m、滑块50%，由用户手动执行。新增可选motion.geometry_tip_timing（仅local.yaml启用）：解析MoveIt及驱动实时URDF，在各段直线关节路径计算笔尖导数上界，包含解析二阶导数余项，取两种模型较大值；MoveIt FK与本地URDF FK须严格一致。保留零端点速度的三次插值、原关节速度/加速度/笔尖速度、跳变拒绝、碰撞及完整率检查。旧配置继续使用2m通用保守计时。

现场同一小脸222点从约2800s降到257.3s（50%滑块预计约515s）。仅完整规划和速度上界检查，未发送真机轨迹，不代表真机执行验收。产物runs/smiley-50mm-geometry-timing-plan/plan.json。驱动工厂标定URDF与MoveIt名义URDF有约1mm差异，计时同时覆盖两者，不宣称消除几何误差。TCP240mm为用户选定估计值，pen_tcp=false；新纸面runs/current-50mm-paper-check.yaml仍verified=false，须实测核验后执行。重夹笔需重新核对TCP及笔身包络。43项Humble回归通过。

## 项目地图

本仓库实现 UR7e 桌面绘画，主要链路：图片→标准化笔画→纸面/笔尖几何→MoveIt Cartesian path→scaled joint trajectory action。README 是用户安装运行入口；docs/calibration.md 是现场测量说明；Code_of_algorithm.md 记录初始设计和参考证据；docs/validation.md 记录已执行检查及未验证边界。

关键路径：src/doodle_ur7e/doodle_ur7e/{image,vision,core,ros,cli}.py；config/lab.yaml；scripts/{build_lab.sh,test_humble.sh,verify_moveit.py}；tests/。构建/日志/标定/人像输出均不跟踪，正式运行到 runs/<name>。

## 环境归属和绝对路径

- a6000 SSH 别名 `msc-a6000`，项目 `/home/msc-auto/Harroldx/Doodle_Ur7e`。这是用户授权创建的目录，未修改相邻课程项目。
- 本地开发副本 `/Users/harrold/Documents/Codex/2026-10-13/d435i-eye-in-hand-camera/work/Doodle_Ur7e`。
- 只读参考副本位于本地同一 work 下的 DoodleDroid 和 eecs106a-fa26；revision 见 Code_of_algorithm.md。
- 测试容器项目挂载为 `/workspace`，源目录相同；不使用 GPU，不连接真实机器人。
- 实验室预期路径 `~/ros_workspaces/Doodle_Ur7e`，这是部署约定，尚未在现场创建或核验；每个终端先 `distrobox enter ros2`。

## 当前目标和授权

用户要求参考 DoodleDroid，适配课程 Lab5 的 UR7e/Humble/腕部 D435i；提前夹笔，纸铺桌面；在 a6000 开发并测试，完成后创建 HARROLDX/Doodle_Ur7e 私人仓库上传，交付上机准备和运行步骤。没有真机远程动作授权，也没有真机可用。本地与服务器测试只使用合成状态/场景。

## 验证与恢复

在 Humble 环境使用 `scripts/test_humble.sh`；真实 MoveIt 离线检查使用 `ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=231 python3 scripts/verify_moveit.py`，先 source Humble 和本项目 install/setup.bash。它启动 MoveIt + robot_state_publisher，使用官方 UR7e 模型和合成 JointState，不启动硬件驱动或发送执行 action。

最新具体结果以 docs/validation.md 为准。未知项：真实工位 UR 包版本、实际 TF 树、笔尖 TCP、相机外参、桌面/纸高、碰撞盒、图像质量、接触力与绘画效果。模板不能当作测量结果。开发环境的 isolated domain 不可抄到真机流程。

## 本轮交付状态

已完成实现、21项回归、实际UR7e MoveIt悬空/接触高度规划及障碍拒绝、图像/打印板预览。详细版本与证据见 docs/validation.md。私人远端 https://github.com/HARROLDX/Doodle_Ur7e 已创建并核验 PRIVATE；初始实现 149e2e0 已推送 main，服务器 Git 与本地同步。测试环境镜像 doodle-ur7e-humble:validated 已保存；其 --network none / UID 1000 复验仍为21项通过。测试容器与规划进程均已退出，无活动任务。

## 2026-10-14：P1/P2 修复

用户授权一起修复两项复审问题。P1：新增 shutdown.py 管理停止信号，CLI 保留 ROS context，Robot.execute 对待接受目标执行有界等待/取消/终态确认。P2：README 提供显式 SwitchController 命令与 active 状态检查，仍不自动切换或开合夹爪。25项测试通过，恢复命令已与Humble消息schema核对，无真实设备动作，测试容器已退出。当前日志位于服务器 runs/stop-fix-tests.log。

Code_of_algorithm.md 为初始基线；其中取消流程描述的具体实现已由本轮增强，最新信号处理与恢复操作以 README、shutdown.py、ros.py 和 docs/validation.md 本节为准。

## 2026-10-14：Isaac Sim 全流程仿真

用户明确要求在原服务器目录搭建、测试、运行、输出视频并推送代码；已明确允许共享 GPU 0。实际镜像 Isaac Sim 5.1.0，官方 Humble UR 包复用。新增 sim/ 场景、资产生成、控制栈运行、录像及评估脚本；src/doodle_isaac_hardware 是实际 ros2_control SystemInterface，对接 Isaac 的实际关节反馈，复用官方 scaled controller。仿真域固定231，未连接真机。

GPU smoke 与完整物理仿真绘画已通过；27项回归、真实控制器取消和1049点碰撞复验通过。相机原点误差0.553mm、实际笔迹到规划路径P95 0.172mm，详见 docs/validation.md。服务器产物 /home/msc-auto/Harroldx/Doodle_Ur7e/runs/isaac；缓存 runs/isaac_runtime。运行容器使用同一 UID 与独立共享 IPC，避免跨用户 DDS、Carb PID 命名共享内存冲突。算法文档仍为初始基线，新仿真说明见 docs/isaac_sim.md。

最终交付：runs/isaac/drawing_60x.mp4、actual_drawing.png、metrics.json。执行及世界均成功结束。正式录像含实际机器人渲染与实测笔尖小窗；简化夹爪、相机外壳、几何墨迹及固定速度缩放的限制见 docs/isaac_sim.md。服务器 ROS 运行是隔离 Docker/Humble，现场仍按 distrobox enter ros2；没有声称服务器已经配置现场 Distrobox 或真机验收完成。

## 2026-10-14：场景外观与工位改进

按用户照片加入腕部固定相机支架、闭合双指夹爪及刚性夹持笔，移除底座垫高块，改为底座落地和独立四脚矮桌。共享几何在 sim/scene_geometry.py；同步更新桌面碰撞、标定独立核对和初始观察姿态，未改变真机配置 lab.yaml。夹爪为照片近似几何，不是确认过型号的 CAD。

旧验证产物保留在服务器 /home/msc-auto/Harroldx/Doodle_Ur7e/runs/isaac_v1_validated。新结果位于 runs/isaac，runner 日志 runs/scene_v2b_runner.log；GPU 0 使用沿用已有授权。完整执行和录像验收已完成，action成功、路径完整率100%，新版视频52.667秒，实际到规划路径P95误差0.2267mm。初始IK、12帧相机标定（误差约0.632mm）及1031点最终碰撞包络复验均通过，具体证据见docs/validation.md。

### Baymax 可选功能（同轮追加需求）

用户要求外来图片直接进入绘画链路；GPT模式先交流、出图，用户确认满意后才进入绘画。实现 `doodle baymax --image` 和 `--chat`，后者以 `/generate` 出图、`/confirm` 确认当前图；模型没有ROS/终端工具权限。继续对话取消待确认图片，原图哈希变化拒绝处理。新增入口共用原有机器人校验和执行流程。

图片库位于服务器 `/home/msc-auto/Harroldx/Doodle_Ur7e/images/{generated,external,processed}`；只提交外来笑脸示例，用户图片/对话/确认记录默认忽略。说明在docs/baymax.md，核心在src/doodle_ur7e/doodle_ur7e/baymax.py。39项测试与实际MoveIt正常/阻挡CLI检查通过，日志runs/baymax_final_tests.log和runs/baymax_moveit。

当前没有配置OPENAI_API_KEY，联网GPT调用未验证，用户需在实际Distrobox终端设置自己的API密钥。代码使用官方Responses端点和用户指定gpt-5.6-sol，图片工具gpt-image-2.5-flare；不读取Codex凭据。Code_of_algorithm.md仍为初始基线，新增功能以本记录及docs/baymax.md为准。

## 2026-10-22：README 演示视频

用户要求在 README 的 Isaac Sim 段落下展示既有仿真录像，并为后续真机录像保留位置。已核对最终版 `UR7e_IsaacSim_v2_60x.mp4`：H.264、1280×720、52.667 秒，SHA-256 为 `96402808c19e909d52734940c96186c42230eee409831c9b366871b8c5dc5382`。仓库内路径为 `docs/assets/ur7e_isaac_sim_drawing_60x.mp4`。初版普通 raw URL 实测只显示链接；已将同一文件上传为本仓库的 GitHub 视频附件，README 改用独立的 `github.com/user-attachments/assets/…` URL 嵌入播放器，并保留仓库文件链接作为回退。真机视频尚不存在，不得把占位说明写成已完成实验。

## 2026-10-28：Lab5 / Lab7 上机复审

参考课程ccdfc55，项目基线6411a61，保留用户后加的视频README。完整对照和验证见docs/lab_review.md，现场入口START.md。新lab模板采用Lab7彩色光学帧，旧local.yaml不自动改；mount内部TF等待60秒；doctor显示笔尖和起始姿态误差。40项Humble回归及实际UR7e MoveIt悬空/接触/障碍拒绝通过；服务器日志runs/lab57_review_20261028.log。仅CPU、隔离网络、无真机动作。Code_of_algorithm.md中以Lab5深度外参为模板的叙述已过时，以docs/calibration.md新旧配置迁移说明为准。

### 最终文档整理

README简化为概览/验证状态/视频/文档索引，START集中完整安装、分终端启动、标定、小方框验收、图片/GPT入口及停止命令。修正重复安装覆盖local.yaml的示例，更新相关链接。最后复核CLI参数、执行门槛、TF/标定/控制器路径，未发现新增需修复代码问题；40项回归再次通过（runs/final_docs_review_20261028.log），复用同日实际MoveIt验证，本轮不改运行逻辑。

## 2026-11-03：夹笔流程和100mm悬空

用户要求先启动环境，再单独toggle_gripper夹笔、freedrive定位，结束手动tuck。START已按此顺序整理，toggle为状态切换须观察，不自动调用夹爪或归位。CLI draw/preview/baymax默认hover改为0.10m，允许范围0..0.10m；起始默认净空115mm。保留接触绘画显式hover=0。测试图为24mm方框和12mm短线。

41项回归通过；实际UR7e MoveIt的100mm悬空88点、接触105点均完整，故意障碍拒绝。日志runs/hover100_final.log。取消测试使用原固定姿态，故显式保留该测试的10mm hover；首次MoveIt检查TF尚未发现，离线测试启动等待改为3秒后完整通过。没有真机动作或GPU运行。旧算法文档默认高度以START及CLI新值为准。

### 分级悬空验收补充

按用户要求将START流程细化为100mm→30mm→可选10mm/中间正高度→接触，每一级先规划再执行并人工验收。明确hover单位、纸面法向、各级起始净空，以及高度不符应复核标定而不是用hover补偿。本轮仅文档变化，命令对照CLI与0..0.10m范围核对，复用前轮41项和MoveIt验证；未声称新增30mm真机测试。

## 2026-11-09：A4 一体式打印纸

新增 `docs/print/ur7e_drawing_board_A4.pdf` 与摆放/打印说明；README、START链接到直接打印成品。`scripts/generate_print_board.py` 从lab.yaml读取paper参数，复用vision.dictionary/marker_points生成矢量marker，无ROS依赖，不改变规划/控制。A4 210×297mm，25mm黑色边长、150mm中心距、120×120mm中央空白区；ID0/1靠底座、3/2远离底座只是建议布局，不把纸面+Y误称base_link+Y。

已渲染目视检查；600dpi成品反向检测恰好ID0..3，角点顺序与运行时一致，最大角点偏差0.039mm（栅格化/检测误差），PDF页尺寸核对通过。检查脚本与结果位于本地 `runs/print-check/`。本轮不涉及真机或GPU，无需重跑运动测试。交付文件另存工作区 `outputs/ur7e_drawing_board_A4.pdf`；打印后仍需用尺核对，打印机实际比例未由软件验证。

## 2026-11-24：课程项目展示 README

以远端 deedd91 的真机操作版为基线，将 README 改为 UC Berkeley EECS 106 期末项目介绍。展示真机笑脸绘画、成品照片、从图像到机器人执行的流程和原 Isaac Sim 演示。START.md 保留该版本的现场操作。
`docs/assets/ur7e_real_drawing_5x.mp4` 为仅视频的 H.264 MP4（544×960，30 fps，63.27 秒）。原有 301.37 秒绘画录像以 5× 播放并在右上角标速，片尾追加 3 秒成品实拍。README 通过同仓库 GitHub 视频附件嵌入播放器，并保留仓库 MP4 链接，照片位于 `docs/assets/ur7e_real_smiley.jpg`。
