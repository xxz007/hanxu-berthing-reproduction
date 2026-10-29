# 实验窗口交接

更新：2026-10-29，America/New_York。完整复现目标仍在进行，按docs/08_full_reproduction_protocol.md的R0–R8顺序执行，未宣称全部完成。

## 范围和授权

当前根为01_projects/hanxu-berthing-reproduction，桌面旧项目与v0.1固定基线保留。理论窗口独立精读／验收。GitHub私有仓库xxz007/hanxu-berthing-reproduction已授权同步代码、文档和生成基线；论文PDF、派生源图／地图及私人笔记不自动上传。新包安装、公开发布或发外部消息另需授权。

用户提供的学位、期刊及2010 G2CBS三份PDF只读，来源清单为docs/source_manifest.json及docs/journal_source_manifest.json，本机路径在忽略入Git的docs/source_local.json。期刊第4–5页提供伪代码、第7页表1与学位论文表3.3一致；坐标书写顺序不同，航向需单独核对。

## 本轮已推进

- R1：全局Zhang–Suen细化恢复绘图中心线，三个预设颜色阈值均保持各自组件／封闭孔洞数，全部参考点自由可达；可见原黑网格的F1约从0.883提高至0.912。未移点／清格。基线1642叶、695阻塞细格。允许作为独立恢复的规划输入，不等于作者原始地图。输出outputs/20261029T164055_994609Z_paper_map_r1/。
- R2：原图恢复地图的普通A*在两个算例×三个阈值上与同图Dijkstra一致，全段闭矩形LOS和长度回算通过。基线通航5742.8m、船艉目标位置4397.1m。输出outputs/20261029T165627_390562Z_paper_search_r2_r3/，含trace及策略差异。
- R3：parent解释为parent(current)。无回退版本6例均不可达；Theta式保留current候选可达，基线4680.1m／3747.4m。不是作者实现等价证明。邻域仍是共边，LOS仍是解析矩形参照，航向／G2CBS／RS尚未加在原图结果上。宽阈值的船艉位置长度会变化，不能忽略重建不确定性。
- 39项标准库测试、9项研究测试通过。最新图outputs/20261029T165630_437056Z_paper_search_plots/paper_search_comparison.png已目视核对。报告reports/R1_R3_原图搜索阶段记录.md；规则解释docs/09_parent_policy_audit.md。

## R4资料与试算（本轮）

用户已提供完整G2CBS 2010原文，8页561–568；SHA256 dff618b4decd5fed48004bfc4b9eba1d31e708335066d82ecfa7c522a8fa072f。已目视核对563–564页式4–15；来源docs/g2cbs_source_manifest.json。本地路径在忽略Git的source_local.json，PDF未复制或上传。43项标准库测试通过，含新增4项公式／G²／曲率／预算／缩放检查；此前9项预处理检查本轮未重复。

outputs/20261029T231707_194868Z_g2cbs_r4_audit含12个原图试算及快照哈希：通航原始9点在121.910m段需209.954m，拒绝重叠；独立LOS删点为6点后曲线检查通过（kmax0.004997920），不能当成“不需预处理”的复现。船艉位置3点平滑会碰障碍，三阈值简化版均失败。未加原图航向／RS，完整R4仍未通过。报告reports/R4_G2CBS原文核查与原图试算.md。

印刷c1按字面构造有厘米级B3/E3残差，原因未确认；原有代码共享中点为显式闭合选择，实际曲率与G²另查，未变更构造行为。原文2010未给韩旭所述多短转向合并完整流程。另从ARAA官方下载2008参考文献27完整10页并目视核对5–7页，见docs/g2cbs_2008_source_manifest.json。式21一次二分后当前短段仍需190.456m，短缺68.546m；只做长度预算，未实现二分曲线。公开PDF仅留忽略的data/raw/。

Theta*完整原文已核查：arxiv.org/pdf/1401.3843印刷542–543页明确LOS失败时用current回退，为现有候选规则提供依据；网格／LOS不同，作者一致性仍未确认，详见docs/09_parent_policy_audit.md。

## 当前下一步

先攻克短段合并与碰撞修正：2008余量／二分方法与韩旭38页的合并不能混同；不得缩小d而不检查曲率，不得以独立LOS删点的成功代替原始点合并验收。再继续R5/R6原图RS组合（需先确认xy／航向变换）、R7方法对比、R8坐标与重规划。目标仍active。无需继续等待学校IEEE入口；2010全文缺失已解除。GSHHG尚未下载，不能与3.2图等同。未发外部邮件、未安装新依赖。

## 保留的旧基线

v0.1合成主链reports/baseline/v0.1与S1基线不变。原粗墨迹失败outputs/20261029T163029_584515Z_paper_map_r0保留。主规划仍为独立标准库Python3.12环境；研究预处理／绘图使用workbench已有NumPy/Pillow/PyMuPDF/SciPy/Matplotlib，无新安装。v0.1约3796.1m／4265.2m、倒车1129.5m属于合成图，不能与本轮原图位置路径混用。
