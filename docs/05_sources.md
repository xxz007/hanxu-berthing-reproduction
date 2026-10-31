# 来源、版本与改写记录

核查日期：2026-10-29，America/New_York。

## 学位论文

韩旭，《船舶靠泊路径规划与模型预测航迹保持控制》，大连海事大学，2023，工学博士。用户本地PDF，SHA-256见 `source_manifest.json`。原文只读，按页提取第三章和参考文献作核对；不将原文、整页截图或私人理论笔记入Git。

## G2CBS

- 原文参考文献127：Yang, K.; Sukkarieh, S. *An Analytical Continuous-Curvature Path-Smoothing Algorithm*. IEEE Transactions on Robotics, 26(3), 561–568 (2010). [DOI](https://doi.org/10.1109/TRO.2010.2042990)。
- 用户现已提供完整8页IEEE原文，561–568页齐全；本地哈希与核查范围见[g2cbs_source_manifest.json](g2cbs_source_manifest.json)。已目视核对563–564页控制点与曲率公式。PDF只读，未复制或上传。
- 初始改写依据：Mithun Nallana 与 Balaji C.A. 的[独立MATLAB实现](https://github.com/MithunNallana/path-smoothing/blob/c610cdf2b7b881c6850343a8f0a8434323ccdfbe/code/matlab/pathsmoothbezier.m)，固定提交 `c610cdf2b7b881c6850343a8f0a8434323ccdfbe`，MIT许可存于 `third_party/path-smoothing.LICENSE`。
- 本项目将其二维对称双三次构造改写为标准库Python，直接在全局二维坐标构造控制点；现已与原文式(4)–(15)交叉检查。印刷常数c1=7.2364按字面计算会有小接缝，原因未确认；代码共享中点属于显式实现选择，另查实际曲率及G²连续性。增加共线处理、航段长度／转角重叠拒绝、曲率数值极值和保守曲线碰撞检查。
- 没有引入完整第三方运行程序；没有照搬其3D转换或固定步长绘图代码。未实现多转角合并，不能声称完整还原论文图3.8的所有情形。

## Reeds–Shepp

- [PythonRobotics解析族源文件](https://github.com/AtsushiSakai/PythonRobotics/blob/8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031/PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py)，固定提交 `8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031`，作者 Atsushi Sakai、Videh Patel 及其他贡献者，MIT许可见 `third_party/PythonRobotics.LICENSE`。
- 保留12个解析族的计算函数；移除NumPy、Matplotlib、路径修改和主程序；角度规范化改用标准库 `math.remainder`。自己的代码完成候选去重、端点残差过滤、正负长度、方向要求、加权代价、解析采样和连续碰撞检查。
- 不沿用上游 `set_path` 的近长度去重规则，避免过早丢弃同族但倒车长度不同的候选。候选生成与采样解耦，小段不会因绘图步长而消失。
- 这是无障碍最短RS解析族产生的有限候选集，筛选碰撞和倒车权重不等同于求解有障碍、加权全局最优RS规划；候选失败不证明任意有障碍曲率路径不存在。
- 原图组合诊断另加独立推导的S+ C+ | S-轴线接入候选，见`stern_axis.py`及R5/R6阶段报告。它由学位论文式3.3的运动基元构造，不属于上游经典最短字族，也不代表作者代码；经典候选失败分开保留。

本轮限定检索未找到可确认属于学位论文作者的官方源程序，也未取得长滩港原始数据。独立库均未声称是作者实现。

## 2026-10-29完整原文核查补充

本轮通过Firecrawl读取大学站点TRO2010.Xu.pdf，7页文件主体是Kai Xu与Nabil Simaan的另一篇文章，末尾只有Yang与Sukkarieh的开篇，并非561–568页全文。它不能支持G2CBS控制点／短航段处理的完整核对。OpenAlex与Semantic Scholar当时未给OA全文地址。此为历史检索失败记录；用户随后提供了完整本地IEEE原文，当前不再等待学校入口。

地图全局中心线细化使用自行实现的Zhang–Suen两子步算法，方法出处https://doi.org/10.1145/357994.358023 ，公开说明https://scikit-image.org/docs/stable/auto_examples/edges/plot_skeleton.html 。不是论文作者提供的地图恢复代码。

## 原文补齐后的边界

2010原文IV节提供对称双三次构造和曲率所需长度；VII-A将删去冗余路径点交给规划器，VII-B提到剩余长度余量并引用2008参考文献27。未找到韩旭38页所述多个短转向自动合并的完整规则。原文VI节的离散碰撞结点方法也不等于本项目的保守曲线细分检查。公式、路径点预处理、短段合并和碰撞修正须分别验收。

已从[ARAA官方会议入口](https://www.araa.asn.au/conference/acra-2008/)定位到同作者2008论文[Planning Continuous Curvature Paths for UAVs Amongst Obstacles](https://www.araa.asn.au/acra/acra2008/papers/pap104s1.pdf)。先经Firecrawl读取到余量分配、角度二分和流程文本，后使用已有科研Python成功下载完整10页PDF，目视核对第5–7页。来源哈希见g2cbs_2008_source_manifest.json。已用式(21)做独立长度预算，一次二分后本例短段仍不足；未实现二分曲线生成，也未宣称短段合并已解决。

## R7参考方法

JPS依据Hu等2021年原文4.1节独立实现，禁止对角穿障碍角点；无JPS+或goal bounding。Smooth-RRT依据用户提供的Yu等2017完整PDF，采用目标偏置、转角限制和贪心折线简化。式5的方向裁剪、根航向及双接头保护为明确补充解释，incoming-only失败保留；未将其折线伪称连续曲率。来源哈希见[r7_sources.json](r7_sources.json)，结果与限制见[R7报告](../reports/R7_方法对照与来源差异.md)。未复制第三方程序。

## 2026-10-30续核与真实数据

2010完整8页PDF已按用户新路径重新定位、核对相同SHA并逐页渲染检查；不是只有开篇或缓存文本。2008二分构造已经实现并检查；本轮新增的相邻同向工程合并另行标记，不能认定为作者规则。参见[g2_merge_sources.json](g2_merge_sources.json)。

新坐标内核与既有PROJ9.7.1独立核对，公式审计见[r8_coordinate_sources.json](r8_coordinate_sources.json)。GSHHG2.3.7官方native binary已下载并核对公布SHA，粗细地图在原生层级填陆和保守闭格缓冲后运行；来源、许可与变换见[r8_data_sources.json](r8_data_sources.json)。没有安装GIS包，GEOS仅用于独立审计。之前“未实现二分／坐标”的语句属于历史阶段。
