# 文献、数据与第三方许可

仓库提供独立实现、配置与生成结果。文献 PDF、原始海图和私人笔记不随代码分发；文献与代码归属不因改写而改变。

## 文献依据

- 韩旭，2023，《船舶靠泊路径规划与模型预测航迹保持控制》，大连海事大学博士学位论文；主要对应第三章。题录、页码与源文件校验信息见 [学位论文元数据](source_manifest.json)。本项目不是作者官方代码。
- Yang, K.; Sukkarieh, S. *An Analytical Continuous-Curvature Path-Smoothing Algorithm*. IEEE Transactions on Robotics, 26(3), 561–568 (2010). [DOI](https://doi.org/10.1109/TRO.2010.2042990)。元数据见 [G2CBS 来源](g2cbs_source_manifest.json)。
- Yang 与 Sukkarieh，*Planning Continuous Curvature Paths for UAVs Amongst Obstacles*，ACRA 2008。[官方会议论文](https://www.araa.asn.au/acra/acra2008/papers/pap104s1.pdf)。短段余量及角度二分依据见 [2008 来源](g2cbs_2008_source_manifest.json)。工程合并的独立实现说明见 [合并来源](g2_merge_sources.json)。
- JPS 与 Smooth-RRT 的文献标识、参数和独立解释见 [方法对照来源](r7_sources.json)。Smooth-RRT 参考 Yu 等 2017 年 *Path Optimization of AUV Based on Smooth-RRT Algorithm*，[DOI](https://doi.org/10.1109/ICMA.2017.8016038)。
- 图像中心线恢复使用独立实现的 Zhang–Suen 两子步细化算法，[DOI](https://doi.org/10.1145/357994.358023)。该预处理不是论文作者提供的地图。

## 第三方代码

- G2CBS 初始双三次构造改写参考 [Mithun Nallana 与 Balaji C.A. 的 MATLAB 实现](https://github.com/MithunNallana/path-smoothing/blob/c610cdf2b7b881c6850343a8f0a8434323ccdfbe/code/matlab/pathsmoothbezier.m)，固定提交 `c610cdf2b7b881c6850343a8f0a8434323ccdfbe`；[MIT 许可](../third_party/path-smoothing.LICENSE)。二维标准库 Python 构造、共享中点及几何检查的选择单独说明，不宣称完全等同原文所有情形。
- Reeds–Shepp 与 Dubins 解析族参考 [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics/tree/8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031)，固定提交 `8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031`，归属 Atsushi Sakai、Videh Patel 及贡献者；[MIT 许可](../third_party/PythonRobotics.LICENSE)。独立代码完成候选去重、端点／碰撞检查、方向、权重与组合接入。Dubins 来源见 [来源记录](dubins_sources_a5.json)。

完整第三方归属见 [third_party/README.md](../third_party/README.md)。

## 地图与坐标数据

- NOAA ENC：固定图幅 `US5LGBCE` / `US5LGBDE`，基础文件与更新共同读取；下载地址、发行版本、哈希、许可和处理规则见 [海图来源清单](open_chart_stage_a_sources.json)。使用者应校验配置要求的版本，不能假设最新在线文件与固定快照相同。
- GSHHG 2.3.7：官方 native binary，保留原生岸线层级，用于粗细地图的独立局部重规划。地址、SHA、许可及变换规则见 [GSHHG 来源清单](r8_data_sources.json)。
- 墨卡托坐标、东／北方向和米／格单位约定及 PROJ 对照依据见 [坐标来源](r8_coordinate_sources.json)。

## 图示

实际成果图由对应绘图脚本读取生成结果绘制。框架图使用 AI 图像生成，属于方法概念示意，不是实验输出；图示说明见 [assets/README.md](assets/README.md)。

结果与方法边界见 [实验说明](experiments.md) 和 [方法说明](methods.md)。
