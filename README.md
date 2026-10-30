# 文献实验

**韩旭（2023）博士论文《船舶靠泊路径规划与模型预测航迹保持控制》**

第三章：基于多尺度 A* 算法的靠泊路径规划 · Python 独立学习复现

**一句话理解：给船舶做一套“靠泊导航”——先在开阔处粗、近岸处细的地图找出避障通路，再把转弯变平滑，安排最后的朝向和进退动作，让船按要求抵达泊位。**

[框架图](#1框架图) · [项目结构](#2项目结构) · [学习路线](#3学习路线) · [文献实验解读](#4文献实验解读) · [成果展示](#5实验复现结果展示) · [完成清单](#6未完成部分说明) · [运行指南](#7项目运行指南)

## 1、框架图

![第三章实验拆解框架：地图输入、四叉树、A*与LOS、G2CBS、RS接入、几何检查，以及方法对照和待完成的坐标转换](docs/assets/chapter3_framework.png)

## 2、项目结构

```text
hanxu-berthing-reproduction/
├── README.md                     项目首页：方法、成果、进度与运行入口
├── configs/                      地图、算法参数、固定随机种子
├── src/berthing_planner/
│   ├── quadtree.py               四叉树与几何查询
│   ├── raster_map.py             恢复地图与规划图接口
│   ├── search.py                 普通 A*、父节点连接改进
│   ├── curves.py                 基本 G2CBS 与曲线检查
│   ├── g2_bisection.py           短航段角度二分构造
│   ├── reeds_shepp.py            RS 接口与候选检查
│   ├── rs_families.py            RS 解析路径族
│   ├── stitching.py              平滑前段与 RS 的组合接入
│   ├── stern_axis.py             船艉轴线接入补充策略
│   ├── jps.py                    JPS 对照
│   ├── smooth_rrt.py             Smooth-RRT 对照
│   └── pipeline.py               合成港口学习流程
├── scripts/                      地图恢复、实验运行、对照与绘图
├── tests/                        标准库算法检查
├── research_tests/               地图预处理检查
├── docs/                         原文对齐、来源、复现协议、首页图示
├── reports/                      阶段报告与固定基线回放
├── third_party/                  第三方代码来源与许可
├── requirements-reconstruction.txt  地图恢复／绘图环境版本记录
├── data/                         本地原始／处理数据目录
└── outputs/                      每次运行的结果、配置、快照与哈希
```

## 3、学习路线

<img src="docs/assets/learning_roadmap.png" alt="五阶段学习路线：先看整体效果、弄清地图和搜索、处理转弯与抵泊、回到论文条件、最后攻克缺口" width="640">

## 4、文献实验解读

### 为什么要把这些方法组合起来？

靠泊规划同时面临三个问题：

1. 地图划得太细会增加搜索量。
2. 直接找到的折线可能转不过去。
3. 抵达终点时还要满足船头朝向和倒车要求。

因此，本章依次处理“地图规模、路径转弯、抵泊姿态”，为后续运动规划与控制提供路径。

### 各个模块具体解决什么？

| 模块与原文位置 | 通俗解释 | 本项目如何拆成实验 |
| --- | --- | --- |
| 四叉树，图 3.1—3.4 | 空旷水域少放节点，靠近障碍时细分，把计算用在需要精细判断的位置 | 恢复障碍地图，检查区块、邻接、端点归属和可达性 |
| A* 与 LOS，图 3.5—3.7 | A* 找整体通路；能直接连通时，减少沿区块逐级连接造成的绕折 | 先用 Dijkstra 核查普通 A*，再比较两种父节点规则解释；当前采用明确标注的 Theta 式回退 |
| G2CBS，图 3.8 | 把保留的折角变成受曲率限制的弯道；转弯空间不足时还需处理相邻转向 | 检查基本构造、角度二分、曲率和碰撞；多个短转向的合并仍未完成 |
| RS，图 3.9—3.13 | 用直线和圆弧组合起终姿态，允许前进、倒车，并惩罚过长倒车 | 检查位姿、半径、方向切换和接入；船艉算例另含明确补充的轴线策略 |
| 坐标与重规划，第 3.3 节 | 将经纬度、平面距离和栅格位置统一；精细海图发现新障碍后修正局部路线 | 已核对来源和公式，轴向／单位待独立验证；尚未实现实验 |



## 5、实验复现结果展示

### 成果图

![实际障碍栅格背景下的通航与船艉抵泊路径，标出前进、倒车、起终点、船头朝向和换向点](docs/assets/berthing_results_with_obstacles.png)

左侧为通航到达，右侧为船艉倒车抵泊。灰色为实验实际使用的恢复障碍栅格，青色为前进、红色虚线为倒车；深色箭头标出起终船头朝向，空心圆为换向点。

![同一恢复地图上各方法的路径长度中位数及曲率约束检查结果](docs/assets/method_lengths.png)

### 关键数据

<table width="840">
  <thead>
    <tr>
      <th width="170">恢复港口图算例</th>
      <th width="125">路径长度</th>
      <th width="115">倒车距离</th>
      <th width="110">方向切换</th>
      <th width="125">最大曲率</th>
      <th width="195">中心线几何检查</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="left">通航到达</td>
      <td align="right"><strong>4908.3 m</strong></td>
      <td align="right">0 m</td>
      <td align="right">0 次</td>
      <td align="right">0.005 /m</td>
      <td align="left">通过</td>
    </tr>
    <tr>
      <td align="left">船艉倒车抵泊</td>
      <td align="right"><strong>4142.3 m</strong></td>
      <td align="right"><strong>989.4 m</strong></td>
      <td align="right">1 次</td>
      <td align="right">0.005 /m</td>
      <td align="left">通过</td>
    </tr>
  </tbody>
</table>

方法对照共 **77 次正式试验、15 次预热**；下表为基线地图中位数。RRT 每组 10 个固定种子，确定性方法各重复 5 次。

<table width="840">
  <thead>
    <tr>
      <th width="275">独立重建的方法</th>
      <th width="125">路径长度</th>
      <th width="115">本机时间</th>
      <th width="325">计时内容</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="left">多尺度 A*＋RS</td>
      <td align="right">4908.3 m</td>
      <td align="right">1.474 s</td>
      <td align="left">搜索、平滑前段、两端 RS 及内部检查</td>
    </tr>
    <tr>
      <td align="left">JPS，10 m 网格</td>
      <td align="right">4705.6 m</td>
      <td align="right">0.136 s</td>
      <td align="left">网格搜索</td>
    </tr>
    <tr>
      <td align="left">Smooth-RRT，30° 转角限制</td>
      <td align="right">6032.0 m</td>
      <td align="right">1.703 s</td>
      <td align="left">树生长＋贪心简化</td>
    </tr>
    <tr>
      <td align="left">Smooth-RRT，60° 转角限制</td>
      <td align="right">6004.5 m</td>
      <td align="right">0.251 s</td>
      <td align="left">树生长＋贪心简化</td>
    </tr>
  </tbody>
</table>

JPS 与同网格普通 A* 最短路一致。JPS 和当前 Smooth-RRT 输出仍是折线，不满足本次连续曲率与起终航向要求；各方法的计时与节点口径也不同，不能据此直接排名靠泊效果或宣称同约束速度优势。[完整方法对照](reports/R7_方法对照与来源差异.md)

## 6、未完成部分说明


### 已完成

- ✅ 来源与地图：完成文献、参数来源登记，以及三阈值港口地图恢复与检查。
- ✅ 地图搜索：实现四叉树、普通／改进 A*，普通 A* 已与同图 Dijkstra 核对。
- ✅ 平滑与组合：实现基本 G2CBS 和角度二分；通航到达、船艉倒车抵泊路径通过当前中心线几何检查。
- ✅ 方法对照：已运行 JPS／Smooth-RRT 独立对照，保存 77 次正式试验与 15 次预热；最近 65 项标准库测试通过。

### 待完成任务

- ☐ 完成 G2CBS 多个相邻短转向的合并，验收图 3.8 对应算例。
- ☐ 核实作者原始地图、搜索规则、RS 接入与倒车权重。
- ☐ 明确作者 Smooth-RRT 曲率处理及评价条件，严格复现表 3.3。
- ☐ 实现并验证经纬度、平面米制与栅格坐标转换。
- ☐ 获取并核查 GSHHG 数据，完成第 3.3 节跨图局部重规划。
- ☐ 完成第三章逐项验收与整体复跑。

### 当前难点

- **转弯空间不足**：短航段难以同时满足转弯半径、平滑与避障要求。
- **部分资料缺失**：原始代码、地图及部分关键参数尚未取得或确认。
- **对照口径没有统一**：Smooth-RRT 的曲率处理、计时和节点统计仍待对齐。
- **坐标与地图需要验证**：坐标轴、单位及跨图数据衔接还需核查。
- **整体连续性需要核验**：RS 接入仍有曲率跳变，局部通过不等于整章验收。

详细进度见[暂停检查点](reports/暂停检查点_2026-10-29.md)，验收标准见[复现协议](docs/08_full_reproduction_protocol.md)。

## 7、项目运行指南

### 环境

| 用途 | 环境与依赖 |
| --- | --- |
| 合成演示、规划算法、标准库测试 | Python **3.12**，无第三方运行依赖，使用项目 `.venv` |
| 论文地图恢复、预处理与绘图 | Python 3.12＋PyMuPDF、NumPy、Pillow、SciPy、Matplotlib；版本见 [requirements-reconstruction.txt](requirements-reconstruction.txt) |

以下命令从项目根目录运行，适用于 macOS／Linux。Windows 将 `.venv/bin/python` 换成 `.venv\Scripts\python.exe`；PowerShell 测试前设置 `$env:PYTHONPATH="src"`。

### 流程 A：先运行可随仓库复跑的合成港口演示

```bash
git clone https://github.com/xxz007/hanxu-berthing-reproduction.git
cd hanxu-berthing-reproduction
python3.12 -m venv --without-pip .venv
.venv/bin/python scripts/check_environment.py
.venv/bin/python scripts/run_demo.py
```

私有仓库需要有访问权限；已有本地项目时跳过克隆，已有 `.venv` 时跳过创建环境。运行后打开新生成的 `outputs/<UTC时间>_full_demo/index.html`，可回放各阶段和进退过程。该流程无需论文 PDF，使用**自建合成港口**。

标准库测试：

```bash
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
```

### 流程 B：从本地论文重建港口场景并运行组合实验

**准备材料与科研环境。**需要与[来源登记](docs/source_manifest.json)哈希一致的学位论文 PDF；文件不同会停止，需重新核对图像与标定。本机已具备科研环境，可直接指定：

```bash
source /Users/omo/Documents/AI-Research-Workbench/activate-research-env.zsh
research_python=/Users/omo/Documents/AI-Research-Workbench/.venv/bin/python
```

其他电脑可另建科研环境；以下是需要安装预处理／绘图库时的准备命令，不影响标准库规划环境：

```bash
python3.12 -m venv venv/reconstruction
venv/reconstruction/bin/python -m pip install -r requirements-reconstruction.txt
research_python=venv/reconstruction/bin/python
```

**按顺序运行。**将第一行改为自己的学位论文路径；本组命令在同一个终端执行。时间标记使每次运行写入新目录。

```bash
thesis_pdf="/填写你的路径/船舶靠泊路径规划与模型预测航迹保持控制_韩旭.pdf"
run_tag=$(date -u +%Y%m%dT%H%M%SZ)
paper_run="outputs/${run_tag}_paper_rerun"

# 1. 提取与标定 → 2. 中心线恢复，生成三种阈值地图
"$research_python" scripts/reconstruct_paper_map.py \
  --source-pdf "$thesis_pdf" --output-dir "${paper_run}_r0"
"$research_python" scripts/refine_paper_map.py \
  --input-dir "${paper_run}_r0" --output-dir "${paper_run}_r1"

# 3. 普通与改进搜索 → 4. 平滑前段与 RS 组合
.venv/bin/python scripts/run_paper_search.py \
  --map-dir "${paper_run}_r1" --output-dir "${paper_run}_r2_r3"
.venv/bin/python scripts/run_paper_combination.py \
  --map-dir "${paper_run}_r1" --search-dir "${paper_run}_r2_r3" \
  --output-dir "${paper_run}_r5_r6"

# 5. 绘制组合路径 → 6. 运行并绘制方法对照
"$research_python" scripts/plot_paper_combination.py --input-dir "${paper_run}_r5_r6"
.venv/bin/python scripts/run_comparison_r7.py \
  --map-dir "${paper_run}_r1" --output-dir "${paper_run}_r7"
"$research_python" scripts/plot_comparison_r7.py --input-dir "${paper_run}_r7"
```

若某一步失败，先检查该步输出与报告，再继续后续命令。组合脚本保留失败候选，不会因此把未完成的短转向合并算作通过。地图预处理检查可用 `"$research_python" -m unittest discover -s research_tests -v` 单独执行。

各阶段在指定的 `outputs/` 子目录保存结果及相应配置、输入／源码哈希；脚本拒绝覆盖已有运行目录。地图恢复、平滑审计与验收细节见[完整复跑协议](docs/08_full_reproduction_protocol.md)。
