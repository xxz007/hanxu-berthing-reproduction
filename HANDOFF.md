# 实验窗口交接

更新：2026-10-29，America/New_York。完整复现目标仍在进行，按docs/08_full_reproduction_protocol.md的R0–R8顺序执行，未宣称全部完成。

## 范围和授权

当前根为01_projects/hanxu-berthing-reproduction，桌面旧项目与v0.1固定基线保留。理论窗口独立精读／验收。GitHub私有仓库xxz007/hanxu-berthing-reproduction已授权同步代码、文档和生成基线；论文PDF、派生源图／地图及私人笔记不自动上传。新包安装、公开发布或发外部消息另需授权。

两份PDF只读，来源清单为docs/source_manifest.json及docs/journal_source_manifest.json，本机路径在忽略入Git的docs/source_local.json。期刊第4–5页提供伪代码、第7页表1与学位论文表3.3一致；坐标书写顺序不同，航向需单独核对。

## 本轮已推进

- R1：全局Zhang–Suen细化恢复绘图中心线，三个预设颜色阈值均保持各自组件／封闭孔洞数，全部参考点自由可达；可见原黑网格的F1约从0.883提高至0.912。未移点／清格。基线1642叶、695阻塞细格。允许作为独立恢复的规划输入，不等于作者原始地图。输出outputs/20261029T164055_994609Z_paper_map_r1/。
- R2：原图恢复地图的普通A*在两个算例×三个阈值上与同图Dijkstra一致，全段闭矩形LOS和长度回算通过。基线通航5742.8m、船艉目标位置4397.1m。输出outputs/20261029T165627_390562Z_paper_search_r2_r3/，含trace及策略差异。
- R3：parent解释为parent(current)。无回退版本6例均不可达；Theta式保留current候选可达，基线4680.1m／3747.4m。不是作者实现等价证明。邻域仍是共边，LOS仍是解析矩形参照，航向／G2CBS／RS尚未加在原图结果上。宽阈值的船艉位置长度会变化，不能忽略重建不确定性。
- 39项标准库测试、9项研究测试通过。最新图outputs/20261029T165630_437056Z_paper_search_plots/paper_search_comparison.png已目视核对。报告reports/R1_R3_原图搜索阶段记录.md；规则解释docs/09_parent_policy_audit.md。

## 当前资料缺口与下一步

R3的完整回退／parent语义仍需核对。R4需G2CBS参考文献127完整原文及短航段合并。大学站点TRO2010.Xu.pdf实际主体为另一篇文章，仅末尾含Yang/Sukkarieh开篇，不能当作全文。OpenAlex及Semantic Scholar均未提供OA全文。已异步询问用户学校图书馆／IEEE入口（DOI10.1109/TRO.2010.2042990），尚未收到；不能擅自使用凭据或发送作者邮件。

后续继续R4曲率／短转角、R5/R6原图RS组合、R7公平方法对比、R8坐标与重规划。GSHHG官方与Zenodo入口已核实，但数据未下载，不能与3.2港口图等同。避免在缺少原文时另换平滑算法并称完整复现。

## 保留的旧基线

v0.1合成主链reports/baseline/v0.1与S1基线不变。原粗墨迹失败outputs/20261029T163029_584515Z_paper_map_r0保留。主规划仍为独立标准库Python3.12环境；研究预处理／绘图使用workbench已有NumPy/Pillow/PyMuPDF/SciPy/Matplotlib，无新安装。v0.1约3796.1m／4265.2m、倒车1129.5m属于合成图，不能与本轮原图位置路径混用。
