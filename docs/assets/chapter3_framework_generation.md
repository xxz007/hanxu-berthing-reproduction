# 第三章实验拆解框架图生成记录

- 用途：README 第 1 部分的概念框架图，非实验结果图。
- 日期：2026-10-30（America/New_York）。
- 工具：内置 `image_gen.imagegen`，新图生成，白色背景；未使用 CLI 或原文图像。
- 输入：根据当前 HANDOFF 与阶段报告编写的下列概念流程；没有上传 PDF、恢复底图或用户原始笔记。
- 输出：`chapter3_framework.png`，与本记录同目录。
- SHA-256：`6aa5b50ddec2112bc22cb71d3a85e030fe1615c8b341fc71b7c5dcaf525493c5`。
- 检查：主流程按 1→2→3→4→5→6 连通；已检查中文标签、模块状态、待完成说明与图片边界。
- 状态依据：R1—R7 阶段报告及项目 HANDOFF；生成图中的示意航迹不具有测量或性能含义。

## 完整提示词

```text
Use case: scientific-educational infographic-diagram.
Create a publication-quality Chinese framework diagram for a GitHub README about independent reproduction of Chapter 3, multiscale A* ship berthing path planning. Landscape 16:10, white background, sharp large readable Simplified Chinese typography, clean flat vector-like design, navy headings, teal completed modules, amber partially completed, pale gray dashed unfinished modules. No decorative photographs, no numerical result charts, no logos, no geographic map. Keep text concise and exact. The figure is a conceptual workflow, NOT experimental output.

Title: "第三章 · 靠泊路径规划实验拆解"
Subtitle: "先找通路，再处理转弯与抵泊姿态"

Main flow with six numbered rounded boxes and clear directional arrows, arranged two rows: first row left-to-right boxes1–3; arrow downward at right then second row right-to-left boxes4–6 (reading labels numbered):
1 "地图输入" / "海图障碍 · 起终位姿" / status "已恢复实验地图"
2 "四叉树多尺度地图" / "开阔处粗 · 近岸处细" / status "已实现并检查"
3 "A* 与 LOS 改进" / "搜索通路 · 减少绕折" / status "已实现，作者规则待对齐"
4 "G2CBS 平滑" / "限制曲率 · 平滑转弯" / status "部分完成：短转向合并待解"
5 "Reeds–Shepp 接入" / "起终朝向 · 前进与倒车" / status "已生成组合路径，含补充策略"
6 "几何检查与成果" / "碰撞 · 位姿 · 曲率 · 方向" / status "通航到达 / 船艉倒车抵泊"
Color boxes1,2,6 teal; boxes3,4,5 amber. Main box4 placed right second row, box5 middle second row, box6 left second row, arrows 3→4→5→6. Small simple quadtree/route/curve/ship line icons optional but prioritize typography.

Below the main workflow, two slim supporting bands:
BandA "方法对照（已建立）" / "同环境对照：多尺度 A*＋RS、JPS、Smooth-RRT" / "计时与约束口径需分别说明"
BandB dashed gray "坐标转换与跨图局部重规划（未实现）" / "经纬度 ↔ 平面米制 ↔ 栅格 · 精细海图发现障碍后修正路径"
Footer: "独立实现 ≠ 作者原始代码；严格论文复现尚未完成"
All seven statuses accurately visible. No invented claims or metrics. Clear spacing, consistent boxes, visually balanced, every label fully inside margins.
```

