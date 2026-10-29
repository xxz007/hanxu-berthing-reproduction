# 实验窗口交接

更新：2026-10-29，America/New_York。

## 最新决定与入口

用户选择实验迁至当前工作区，先整体跑通再局部攻坚，理论窗口独立学习和验收。项目根为 `01_projects/hanxu-berthing-reproduction`，桌面旧项目保持不动。当前主入口README、scripts/run_demo.py与reports/整体流程基线_v0.1.md。

GitHub私有仓库：https://github.com/xxz007/hanxu-berthing-reproduction；main为实验代码与固定证据分支。公开性已通过GitHub返回的isPrivate=true核验。

已授权使用明确标注的替代港口地图、LOS父节点候选规则；已授权上传为用户GitHub私有项目，论文PDF／私人笔记不上传。新依赖安装或公开发布另询问。

## 已完成

完整复现已按docs/08_full_reproduction_protocol.md启动。R1原图恢复脚本scripts/reconstruct_paper_map.py和配置configs/paper_map_r0.json已运行；使用既有workbench的PyMuPDF/NumPy/Pillow，无新安装，项目标准库环境不变。5项research_tests通过。当前输出outputs/20261029T163029_584515Z_paper_map_r0/：约7440.98×4572.04m、深度7、2629叶，标定刻度最大残差7.24m/3.76m（不是总地图误差）。起点与船艉终点可连通，但通航终点被蓝色墨迹保守聚合阻塞，三阈值均如此；R1尚未放行，不记原图A*或完整复现通过。详见reports/R1_原图恢复初检.md。下一步核对点/线绘制宽度、中心位置、根边界和终点接入；不可移动终点或强制清格凑通过。源图和派生地图仅留忽略入Git的outputs。

独立Python3.12环境，标准库运行，无pip／第三方运行依赖。沿用S1；新增均匀／多尺度A*、Theta风格父节点候选、独立LOS简化、G2CBS基本转角对、RS解析族和有倒车代价的位姿衔接，双算例HTML回放与阶段图。

35项测试通过，含40对固定种子随机RS位姿。默认两算例长度约3796.1m／4265.2m；后者倒车1129.5m，一次方向切换、末段倒车。起终位姿、曲率数值极值、解析中心线碰撞和连接检查通过。固定配置、指标、轨迹、原始基元和代码快照在reports/baseline/v0.1，S1基线在reports/baseline/s1。

## 限制与下一步

用户已提供本地期刊PDF jmse-11-00153-v2.pdf；已核对12页、SHA-256，并目视核对第4–5页伪代码。来源清单docs/journal_source_manifest.json，对齐记录docs/07_journal_v2_alignment.md，本机路径仅存忽略入Git的docs/source_local.json。期刊表1与学位论文表3.3一致；期刊位姿(4138,417,0)→(2548,3777,0)与学位论文前两项顺序不同，转换及零航向需核对。原文PDF未复制或上传；本次没有改动算法或v0.1结果。

进一步完整复现的核查：已从PDF提取并目视核对图3.3（524×341）和图3.4（950×684），可尝试原图障碍恢复；图3.4横轴y、纵轴x，必须重新标定。尚未实施分割／标定／原图规划。相关期刊论文DOI为10.3390/jmse11010153，后续Firecrawl已成功读取全文，Data Availability Statement为“Not applicable.”；正文未检出代码／数据下载或补充材料入口，但提供A*与父节点修改伪代码。GitHub限定查询尚未发现可确认的作者工程；通用组件与现有私有仓库均不是作者完整源码，图像重建也不是原始数据。详见docs/06_reproduction_feasibility.md；严格同值复现仍需作者资料，v0.1保留不动。

不是长滩港原图或表3.3数值复现；未实现作者完整父节点规则、G2CBS短转角合并、3.3节坐标转换／重规划、Smooth-RRT／JPS。RS连接可有曲率跳变，未做船体和动力学，回放不是真实航行仿真。G2CBS原始2010全文未成功获取，构造依据明确的MIT独立实现；来源与许可见docs/05_sources.md。

下一步让用户先运行整体报告，再选择从S1/S2逐步拆解；每次只改一个参数并保存新输出。理论精读和验收进度不由实验成功代替。
