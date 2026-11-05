"""Portable result report from audited scalar outputs."""


def render(x):
    a=x['nominal_audit']; pairs=a['distance_pairs']
    rows='\n'.join(f"| {r['case']} | {r['constraint_outcome']} | {r['maximum_position_error_m']:.4f} | {100*r['coverage_lower_ratio']:.6f}% |" for r in x['conditions'])
    return f'''# 协同覆盖与实际轨迹实验结果

从固定 NOAA ENC 原始快照重建地图、覆盖任务、曲线与共同时间表，再运行反馈仿真和独立核查。

任务时长 **{x['mission_end_s']/3600:.6f} h**，共 **{x['scan_count']}** 个扫描时段；USV **{x['usv_transfer_count']}** 次转场、航程 **{x['usv_length_m']:.6f} m**。

名义实际覆盖率独立下界 **{100*a['independent_coverage_lower_ratio']:.6f}%**；支援距离上界 **{max(v['maximum_distance_upper_m'] for k,v in pairs.items() if k.startswith('usv__')):.6f} m**，平台间距下界 **{min(v['minimum_distance_lower_m'] for v in pairs.values()):.6f} m**，障碍间距下界 **{min(a['independent_hazard_clearance_lower_m'].values()):.6f} m**。

| 确定性条件 | 约束核查 | 最大位置误差 / m | 实际覆盖下界 |
| --- | --- | ---: | ---: |
{rows}

`certified_pass` 表示保守界通过所有门槛；`witnessed_violation` 表示已有违约见证。数学审计通过并不意味着条件安全。每个条件只有一次确定性仿真，无随机总体或统计显著性结论。

成果图见 `../figures/actual_tracking.png` 与 `../figures/condition_outcomes.png`；实际轨迹回放见 [replay.html](replay.html)，完整核查与来源哈希见 `../validation/` 和 `../manifest.json`。

模型为全驱动二维加速度双积分器与独立航向，理想地速测量、恒定漂流；没有真实欠驱动水动力、推力/能耗、深度、声学、论文 MPC 或实艇验收。严格原论文复现及个人/导师验收仍未完成。
'''
