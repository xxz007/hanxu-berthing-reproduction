# 配置

- `harbor_demo.json`：当前完整流程的可运行合成港口，两算例。原文值与工程选择写在provenance。
- `s1_teaching_map.json`：8×8教学地图，run_s1.py使用。
- `paper_ch3_reference.json`：原文参数定位表，参考用，不是可直接运行的配置。

复制配置作新实验，用 `run_demo.py --config <文件>`。每次先只改一个变量；参数不满足可行性时明确报错。不要把空缺的论文参数填成猜测值。
