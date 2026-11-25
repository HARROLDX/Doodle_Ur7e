# A4 一体式绘画纸

直接下载 [A4 PDF](ur7e_drawing_board_A4.pdf)，单面打印在普通白色 A4 纸上；四个 marker 与中间空白绘图区在同一张纸上，可直接在中央绘画，不需要另盖一张纸。

## 打印与尺寸

选择 **A4、纵向、实际大小 / 100%**；关闭“适应页面 / 缩放到可打印区域”，不要截图打印。黑白打印即可，不需无边距打印。打印后用尺核对底部 50 mm 标尺、marker 黑色外边长 25 mm，以及相邻 marker 中心距离 150 mm；尺寸不符先修正打印缩放。

PDF 由 `config/lab.yaml` 的 `paper` 配置与 `vision.py` 的同一个字典和角点函数生成：

| 参数 | 对应值 |
|---|---|
| 纸张 | A4，210 × 297 mm |
| ArUco 字典 | `DICT_4X4_50` |
| marker ID | 左下 0、右下 1、右上 2、左上 3 |
| `marker_size` | 0.025 m；黑色外边长，不含白边 |
| `width` / `height` | 0.150 / 0.150 m；marker 中心间距，不是 A4 外形尺寸 |
| `margin` | 0.015 m；中央有效绘图区为 120 × 120 mm |

浅灰虚线在有效绘图区外侧 0.6 mm，避免将边界线画进有效区域；空白内部没有预印图案。若现场 `config/local.yaml` 改过上述纸面参数，须与本表一致才能用此 PDF，不能把 width/height 改成 A4 的 0.210/0.297。

## 如何摆放

站在底座一侧看向纸面，按以下方向摆放：

```text
           远离底座 / 纸面 +Y / 图像上方
                ID 3          ID 2
                    中央绘图区
                ID 0          ID 1
           靠近底座 / 图像下方
                     机器人底座
```

- 将纸放在机器人前方可达的平整桌面上，**ID 0、1 的边靠近底座，ID 3、2 的边远离底座**。PDF 上的箭头指向远离底座的方向；它也是所画图像的“上方”。
- 纸面原点在 ID 0 中心，+X 从 ID 0 指向 ID 1，+Y 从 ID 0 指向 ID 3，+Z 从纸面朝上。**这不是对机器人 base_link +X/+Y 的命名**：底座坐标方向要看现场 TF/RViz，不能仅由机械臂外观判断。
- 这是便于操作的建议朝向，不是算法要求的固定朝向。四码标定估计纸面相对底座的位置和旋转；纸张有平面内旋转也能识别，但实际摆放仍需满足 workspace、可达性、姿态和碰撞检查。
- 固定纸张四边，胶带不要盖住 marker 或其周围白边；保持平整、四码同时可见。直接画在这张纸上可避免叠纸高度差。换纸或移动纸张后重新标定，再按 START 的分级悬空流程验收。

## 重新生成（通常不需要）

仓库已提供成品，打印不需要安装任何 Python 包。维护者可在具备 OpenCV ArUco、NumPy、SciPy、PyYAML 和 ReportLab 的独立 Python 环境中，于仓库根目录执行：

```bash
python3 scripts/generate_print_board.py
# 使用不同纸面配置时，重新生成并重新标定：
python3 scripts/generate_print_board.py --config config/local.yaml --output runs/custom_board_A4.pdf
```

此工具不导入 ROS、不连接机器人；未改变运行时规划或控制逻辑。原 `doodle board` SVG 命令仍可使用。
