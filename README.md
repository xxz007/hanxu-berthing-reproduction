# 科研工作汇报

**2026年11月10日组会** · 统计区间11月3日至10日，完成记录截至11月9日。

| 协同任务 | USV支援改进 | 误差条件核查 |
| :--- | :--- | :--- |
| 89个扫描窗口，共同任务约18.826 h | 航程减少93.26%，转场11次降至2次 | 七类确定性条件，五类通过、两类违约 |

## 一 协同覆盖实验与成果

### 完整协同任务

**任务：** 将两台UUV的扫描、等待与一台USV的支援安排放到共同时间轴上，核查任务是否能一起完成。

**成果：** 完成89个扫描窗口的统一安排；最终名义反馈运行的实际覆盖率保守下界约为**99.90392%**，满足99.9%要求。

**35秒加速回放** · 展示扫描过程和两次USV转场。[打开MP4视频](meeting-assets-20261110/mission-accelerated.mp4)

[![协同任务35秒加速回放](meeting-assets-20261110/mission-accelerated.gif)](meeting-assets-20261110/mission-accelerated.gif)

**完整任务轨迹** · 两台UUV扫描，一台USV支援。

[![协同任务实际轨迹](meeting-assets-20261110/mission-trajectory.png)](meeting-assets-20261110/mission-trajectory.png)

### 支援策略改进

**任务：** 在相同UUV任务、初始条件和安全要求下，满足2300 m支援范围，并减少USV往返移动。

**成果：** 优先延长驻守、减少转场，USV支援航程从**10.173 km降至0.686 km**，共同任务时长与规划覆盖下界保持。

[![USV支援策略改进前后路线对照](meeting-assets-20261110/support-comparison.png)](meeting-assets-20261110/support-comparison.png)

| 指标 | 原移动方案 | 改进移动方案 |
| :--- | ---: | ---: |
| USV支援航程 | 10.173 km | **0.686 km** |
| 转场次数 | 11次 | **2次** |
| USV运动时间 | 4364.18 s | **351.97 s** |
| 共同任务时长 | 18.826 h | 18.826 h |

**支援范围与移动代价** · 原驻守方案在2300 m要求下超距；原移动方案虽然可行，但航程约为驻守的15.64倍，进一步推动了上述改进。

[![支援范围与移动代价对照](meeting-assets-20261110/support-range.png)](meeting-assets-20261110/support-range.png)

### 反馈执行与适用边界

**任务：** 检查实际轨迹跟踪误差，以及误差会不会破坏覆盖、避障、平台间距和支援距离要求。

**成果：** 完成0.25 s、0.5 s、1 s三种控制周期的名义核查，并复跑0.5 s名义条件。下图展示全程误差量级及最大峰值附近的原始曲线。

[![名义反馈跟踪误差及局部峰值](meeting-assets-20261110/tracking-error.png)](meeting-assets-20261110/tracking-error.png)

**七类条件对照：五类通过，两类保留违约结果。** 5 m测量偏差出现明确覆盖违约；超速度漂流条件下无法持续跟踪固定参考。

[![七类确定性条件的约束结果与最大位置误差](meeting-assets-20261110/condition-outcomes.png)](meeting-assets-20261110/condition-outcomes.png)

**完整交付：** 已形成统一入口、配置、结果报告与本地交互回放，保存了从固定原始海图复跑的核查记录。

结果限于声明的**全驱动二维替代模型**。毫米量级误差是理想测量和固定模型下的数值结果；每类条件仅一次确定性运行。航程减少尚不等同于真实节能，现有结果尚未经过真实欠驱动水动力、真实声学通信或实艇验证。

## 二 文献阅读与控制基础学习

**任务：** 把现有规划实验放回自主作业系统，厘清规划、制导与控制的关系，并按组会反馈补齐船舶运动基础。

**成果：** 梳理两篇博士论文的框架，形成规划与跟踪认知导图；明确现有实现与后续需要学习的模块。

| 阅读材料 | 本周整理的认识 |
| :--- | :--- |
| 李纪强《海事搜寻任务下机/船协同制导与鲁棒控制》 | 协同参考生成、通信约束与鲁棒执行的分工 |
| 曲星儒《面向区域搜索的UUV集群协同路径跟踪控制研究》 | 区域搜索、协同路径跟踪及制导控制的衔接 |

[![路径规划与跟踪的认知导图](meeting-assets-20261110/research-framework.png)](meeting-assets-20261110/research-framework.png)

- **基础学习已启动：** 11月7日梳理三个学习阶段，11月9日开始第一讲，学习坐标系、船舶运动自由度、运动学与动力学，以及Nomoto艏向响应。
- **下一步实践：** PID艏向控制基线 → 同条件模糊控制对照 → LOS路径跟随，再逐步加入海流和输入约束。

两篇论文目前用于框架阅读，具体制导律与控制算法尚未接入；上述PID、模糊控制及LOS仿真属于后续计划。

<details>
<summary>材料入口与复跑记录</summary>

- [本次汇报的原图与动画](meeting-assets-20261110/)
- [正式实验项目](https://github.com/xxz007/hanxu-berthing-reproduction/tree/main)
- [实验与方法说明](https://github.com/xxz007/hanxu-berthing-reproduction/blob/main/docs/experiments.md) · [最终数值结果](https://github.com/xxz007/hanxu-berthing-reproduction/blob/main/docs/experiment_results.json)
- [反馈执行与固定条件说明](https://github.com/xxz007/hanxu-berthing-reproduction/blob/main/docs/closed_loop_results.md)

11月5日保存的集成复跑记录：39个功能计算与核查步骤、209项核心测试、126项科研检查，以及451份输出哈希核对通过；45份轨迹派生文件与固定基线一致。本次仅整理汇报，未新增科研仿真。课程作业不计入科研工作量。

</details>
