# 来源、版本与改写记录

核查日期：2026-10-29，America/New_York。

## 学位论文

韩旭，《船舶靠泊路径规划与模型预测航迹保持控制》，大连海事大学，2023，工学博士。用户本地PDF，SHA-256见 `source_manifest.json`。原文只读，按页提取第三章和参考文献作核对；不将原文、整页截图或私人理论笔记入Git。

## G2CBS

- 原文参考文献127：Yang, K.; Sukkarieh, S. *An Analytical Continuous-Curvature Path-Smoothing Algorithm*. IEEE Transactions on Robotics, 26(3), 561–568 (2010). [DOI](https://doi.org/10.1109/TRO.2010.2042990)。
- 找到[大学站点PDF线索](https://rii.sjtu.edu.cn/images/6/65/TRO2010.Xu.pdf)，本轮完整PDF获取失败；不能称为已完成原文全文核对。
- 实际构造依据：Mithun Nallana 与 Balaji C.A. 的[独立MATLAB实现](https://github.com/MithunNallana/path-smoothing/blob/c610cdf2b7b881c6850343a8f0a8434323ccdfbe/code/matlab/pathsmoothbezier.m)，固定提交 `c610cdf2b7b881c6850343a8f0a8434323ccdfbe`，MIT许可存于 `third_party/path-smoothing.LICENSE`。
- 本项目将其二维对称双三次构造改写为标准库Python，直接在全局二维坐标构造控制点；增加共线处理、航段长度／转角重叠拒绝、曲率数值极值、几何连续性和曲线碰撞检查。
- 没有引入完整第三方运行程序；没有照搬其3D转换或固定步长绘图代码。未实现多转角合并，不能声称完整还原论文图3.8的所有情形。

## Reeds–Shepp

- [PythonRobotics解析族源文件](https://github.com/AtsushiSakai/PythonRobotics/blob/8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031/PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py)，固定提交 `8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031`，作者 Atsushi Sakai、Videh Patel 及其他贡献者，MIT许可见 `third_party/PythonRobotics.LICENSE`。
- 保留12个解析族的计算函数；移除NumPy、Matplotlib、路径修改和主程序；角度规范化改用标准库 `math.remainder`。自己的代码完成候选去重、端点残差过滤、正负长度、方向要求、加权代价、解析采样和连续碰撞检查。
- 不沿用上游 `set_path` 的近长度去重规则，避免过早丢弃同族但倒车长度不同的候选。候选生成与采样解耦，小段不会因绘图步长而消失。
- 这是无障碍最短RS解析族产生的有限候选集，筛选碰撞和倒车权重不等同于求解有障碍、加权全局最优RS规划；候选失败不证明任意有障碍曲率路径不存在。

本轮限定检索未找到可确认属于学位论文作者的官方源程序，也未取得长滩港原始数据。独立库均未声称是作者实现。

## 2026-10-29完整原文核查补充

本轮通过Firecrawl读取大学站点TRO2010.Xu.pdf，7页文件主体是Kai Xu与Nabil Simaan的另一篇文章，末尾只有Yang与Sukkarieh的开篇，并非561–568页全文。它不能支持G2CBS控制点／短航段处理的完整核对。OpenAlex与Semantic Scholar的该DOI记录本轮均未给OA全文地址；已询问用户学校图书馆或IEEE入口。

地图全局中心线细化使用自行实现的Zhang–Suen两子步算法，方法出处https://doi.org/10.1145/357994.358023 ，公开说明https://scikit-image.org/docs/stable/auto_examples/edges/plot_skeleton.html 。不是论文作者提供的地图恢复代码。
