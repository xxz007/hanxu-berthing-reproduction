# 项目图示

- `actual_tracking.*` 与 `condition_outcomes.*` 为默认集成实验从实际轨迹及独立条件核查生成的成果，提供 PNG/SVG，运行目录同时生成 PDF。数据来源与校验见 [集成结果](../experiment_results.json)。

- 港口路线、跨图重规划、覆盖与 USV/UUV 对照图由 `scripts/` 中相应绘图入口读取实际结果生成。图中轨迹采样用于显示，安全结论以独立几何／连续时间核查为准。
- `chapter3_framework_r8.png` 为 AI 生成的方法框架示意，不是数值实验数据。
- 其他合成地图或学习示意图应按各自标题理解，不能视为真实海图实验。

定量条件与数值见 [实验说明](../experiments.md)，文献、数据与代码许可见 [来源说明](../sources.md)。
