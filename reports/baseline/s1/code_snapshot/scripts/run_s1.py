"""Run the synthetic quadtree experiment; keep every run in a new directory."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
from html import escape
import json
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from berthing_planner.quadtree import QuadTree


def render_map(tree, labels, selected, neighbors, lines, mode):
    """Render the same geometry for the partition, adjacency and LOS panels."""
    x0, y0, x1, y1 = tree.bounds
    scale = min(430 / (x1-x0), 430 / (y1-y0))
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 550 540" role="img" '
             'aria-label="四叉树分区，x向右、y向上"><rect width="550" height="540" fill="#fff"/>']
    for cell in tree.leaves.values():
        a,b,c,d = cell.bounds
        fill = '#333d4b' if cell.blocked else '#f1f5f9'
        if mode == 'neighbors' and not cell.blocked:
            fill = '#9bdad0' if cell.code in neighbors else fill
            fill = '#f9d67a' if cell.code == selected else fill
        parts.append(f'<rect x="{60+(a-x0)*scale:g}" y="{470-(d-y0)*scale:g}" '
                     f'width="{(c-a)*scale:g}" height="{(d-b)*scale:g}" '
                     f'fill="{fill}" stroke="#718096" stroke-width="1.2"/>')
        if mode != 'los':
            x,y = cell.center
            color = 'white' if cell.blocked else '#243247'
            parts.append(f'<text x="{60+(x-x0)*scale:g}" y="{475-(y-y0)*scale:g}" '
                         f'text-anchor="middle" fill="{color}" font-size="13">{labels[cell.code]}</text>')
    if mode == 'neighbors':
        a,b = tree.leaves[selected].center
        for code in neighbors:
            c,d = tree.leaves[code].center
            parts.append(f'<line x1="{60+(a-x0)*scale:g}" y1="{470-(b-y0)*scale:g}" '
                         f'x2="{60+(c-x0)*scale:g}" y2="{470-(d-y0)*scale:g}" '
                         'stroke="#087f74" stroke-width="3"/>')
    if mode == 'los':
        for index, line in enumerate(lines[:3]):
            color = '#a66b16' if index == 2 else ('#087f74' if line['exact_los'] else '#c44747')
            a,b = line['start']; c,d = line['end']
            parts.append(f'<line x1="{60+(a-x0)*scale:g}" y1="{470-(b-y0)*scale:g}" '
                         f'x2="{60+(c-x0)*scale:g}" y2="{470-(d-y0)*scale:g}" '
                         f'stroke="{color}" stroke-width="3"/>')
            for x,y in (line['start'],line['end']):
                parts.append(f'<circle cx="{60+(x-x0)*scale:g}" cy="{470-(y-y0)*scale:g}" '
                             f'r="4" fill="{color}"/>')
    # Axis ticks use equal geometric spacing even for rectangular maps.
    for index in range(9):
        x=x0+(x1-x0)*index/8; y=y0+(y1-y0)*index/8
        parts.append(f'<text x="{60+(x-x0)*scale:g}" y="495" text-anchor="middle" font-size="12">{x:g}</text>')
        parts.append(f'<text x="45" y="{474-(y-y0)*scale:g}" text-anchor="end" font-size="12">{y:g}</text>')
    parts.append('<text x="505" y="495" font-size="15">x</text><text x="40" y="25" font-size="15">y</text></svg>')
    return ''.join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/s1_teaching_map.json')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    config_bytes = args.config.read_bytes()
    config = json.loads(config_bytes)
    # Algorithm semantics are fixed for S1; user configs can change map and queries.
    supported_conventions = {
        'root_depth': 0, 'ownership': 'left_bottom_closed_right_top_open',
        'split': 'split_if_positive_area_obstacle_overlap_and_depth_below_max',
        'mixed_leaf': 'conservatively_block_entire_leaf',
        'quadrant_codes': {'00':'bottom_left','10':'bottom_right','01':'top_left','11':'top_right'},
        'neighbor': 'free_leaves_sharing_positive_length_edge_geometric_scan',
        'los': 'segment_vs_closed_blocked_leaf_rectangles', 'obstacle_contact': 'blocked',
        'implementation_status': 'teaching_reference_not_author_code',
    }
    if config['schema_version'] != 1 or config['conventions'] != supported_conventions:
        raise ValueError('Unsupported S1 schema or conventions; do not silently change semantics')
    if config['coordinate_system']['x_axis'] != 'right' or config['coordinate_system']['y_axis'] != 'up':
        raise ValueError('S1 uses x right, y up')
    if config['data_kind'] != 'synthetic_teaching_map_not_paper_harbor' or config['random_seed'] is not None:
        raise ValueError('S1 is a deterministic synthetic teaching experiment')
    tree = QuadTree(**config['map'])
    leaves = list(tree.leaves.values())
    labels = {cell.code: f'L{index:02d}' for index,cell in enumerate(leaves)}
    rows = []
    for cell in leaves:
        rows.append({'label':labels[cell.code], 'code':cell.code, 'depth':cell.depth,
                     'x_min':cell.bounds[0], 'y_min':cell.bounds[1],
                     'x_max':cell.bounds[2], 'y_max':cell.bounds[3],
                     'center_x':cell.center[0], 'center_y':cell.center[1], 'blocked':cell.blocked})
    points = []
    for point in config['point_queries']:
        cell = tree.locate(point)
        points.append({'point':point,'leaf':cell.code,'label':labels[cell.code],
                       'bounds':cell.bounds,'blocked':cell.blocked})
    selected = tree.locate(config['neighbor_query'])
    if selected.blocked:
        raise ValueError('neighbor_query must select a free leaf')
    neighbor_cells = tree.neighbors(selected.code)
    adjacency = {cell.code:[n.code for n in tree.neighbors(cell.code)] for cell in leaves if not cell.blocked}
    lines = []
    for query in config['los_queries']:
        clear = tree.line_of_sight(query['start'],query['end'])
        endpoint_clear = all(not tree.locate(point).blocked for point in (query['start'],query['end']))
        lines.append({**query, 'exact_los':clear, 'endpoints_only_says_clear':endpoint_clear})
    area = sum((c-a)*(d-b) for a,b,c,d in (cell.bounds for cell in leaves))
    overlaps = 0
    for index, first in enumerate(leaves):
        a,b,c,d = first.bounds
        for second in leaves[index+1:]:
            l,s,r,t = second.bounds
            overlaps += max(a,l) < min(c,r) and max(b,s) < min(d,t)
    x0,y0,x1,y1 = tree.bounds
    metrics = {
        'scope':'S1_synthetic_map_only_no_path_planning',
        'leaf_count':len(leaves), 'free_leaf_count':sum(not cell.blocked for cell in leaves),
        'blocked_leaf_count':sum(cell.blocked for cell in leaves),
        'leaf_area_sum':area, 'map_area':(x1-x0)*(y1-y0), 'interior_overlap_pairs':overlaps,
        'uniform_cells_at_same_depth':4**tree.max_depth,
        'selected_neighbor_centers':[cell.center for cell in neighbor_cells],
        'free_graph_undirected_edges':sum(map(len,adjacency.values()))//2,
        'adjacency_symmetric':all(a in adjacency[b] for a,bs in adjacency.items() for b in bs),
        'los_results':lines,
        'note':'叶区域数差异是表示差异，不是实测内存或速度提升。',
    }
    instant = datetime.now(timezone.utc)
    output = args.output_dir or ROOT/'outputs'/instant.strftime('%Y%m%dT%H%M%S_%fZ_s1_quadtree')
    output.mkdir(parents=True, exist_ok=False)
    (output/'config.json').write_bytes(config_bytes)
    for name,value in [('metrics.json',metrics),('queries.json',points),('adjacency.json',adjacency)]:
        (output/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    with (output/'leaves.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    status=subprocess.run(['git','-C',str(ROOT),'status','--porcelain'],capture_output=True,text=True)
    head=subprocess.run(['git','-C',str(ROOT),'rev-parse','--verify','HEAD'],capture_output=True,text=True)
    source_hashes={}
    for name in ['scripts/run_s1.py','src/berthing_planner/__init__.py',
                 'src/berthing_planner/quadtree.py','tests/test_quadtree.py','tests/test_s1_runner.py']:
        source=(ROOT/name).read_bytes()
        source_hashes[name]=hashlib.sha256(source).hexdigest()
        snapshot=output/'code_snapshot'/name
        snapshot.parent.mkdir(parents=True,exist_ok=True)
        snapshot.write_bytes(source)
    environment={
        'started_at_utc':instant.isoformat(), 'python':platform.python_version(),
        'python_executable':sys.executable,'platform':platform.platform(),
        'third_party_dependencies':[], 'git_head':head.stdout.strip() if head.returncode==0 else None,
        'git_status_porcelain':status.stdout.splitlines() if status.returncode==0 else None,
        'config_sha256':hashlib.sha256(config_bytes).hexdigest(), 'source_sha256':source_hashes,
        'command':sys.argv,'source_kind':config['data_kind'],
    }
    (output/'environment.json').write_text(json.dumps(environment,ensure_ascii=False,indent=2)+'\n')
    maps=[render_map(tree,labels,selected.code,set(adjacency[selected.code]),lines,mode)
          for mode in ['partition','neighbors','los']]
    (output/'quadtree.svg').write_text(maps[0])
    panels=''.join(f'<section><h2>{title}</h2>{svg}<p>{description}</p></section>'
                   for title,svg,description in zip(
                       ['01 · 多尺度分区','02 · 大小区域邻接','03 · 线段碰撞核查'],maps,
                       ['深色为障碍叶。标签对应下方叶表；只细分含障碍的区域。',
                        '黄色为选中叶，青色为共享正长度边界的空闲邻居；只碰角不算邻居。',
                        '绿色表示可见、红色表示受阻；棕色为第3条查询，用于对照端点与整段检测。此图只画前三条查询。']))
    table=''.join('<tr>'+''.join(f'<td>{escape(str(value))}</td>' for value in row.values())+'</tr>' for row in rows)
    query_table=''.join(f'<tr><td>{escape(str(q["point"]))}</td><td>{q["label"]}</td><td>{q["blocked"]}</td></tr>' for q in points)
    los_table=''.join(f'<tr><td>{escape(q["name"])}</td><td>{escape(str(q["start"]))} → {escape(str(q["end"]))}</td>'
                      f'<td>{q["exact_los"]}</td><td>{q["endpoints_only_says_clear"]}</td></tr>' for q in lines)
    html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>S1 四叉树教学实验</title><style>
body{margin:0;background:#edf1f5;color:#213047;font:15px/1.7 -apple-system,BlinkMacSystemFont,"PingFang SC",sans-serif}
main{max-width:1480px;margin:auto;padding:32px}h1{font-size:32px;line-height:1.3;margin:0 0 12px}h2{font-size:19px}
.tag{color:#087f74;font-weight:700}.panels{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:18px}
section{background:white;border-radius:14px;padding:20px;margin:20px 0;box-shadow:0 3px 15px #142d4410}svg{width:100%;height:auto}
table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:8px 10px;border-bottom:1px solid #dde4eb;text-align:left;white-space:nowrap}
.scroll{overflow:auto}code{background:#e8edf2;padding:2px 5px;border-radius:4px}p{margin:8px 0}
@media(max-width:950px){.panels{grid-template-columns:1fr}main{padding:18px}.panels section{margin:0}h1{font-size:27px}}
</style><main><p class="tag">韩旭博士论文 · 第三章 · 实验 S1</p><h1>从单障碍地图到四叉树</h1>
<p>合成教学地图 · x向右、y向上 · 教学长度单位 · 不是长滩港复现结果</p>'''
    html+=f'<p>{len(leaves)} 个叶区域 / {metrics["free_leaf_count"]} 个空闲 / {metrics["blocked_leaf_count"]} 个障碍；同深度均匀分区为 {metrics["uniform_cells_at_same_depth"]} 个单元。</p>'
    html+=f'<div class="panels">{panels}</div><section><h2>位置查询</h2><div class="scroll"><table><tr><th>坐标</th><th>叶标签</th><th>障碍</th></tr>{query_table}</table></div></section>'
    html+=f'<section><h2>完整叶表</h2><div class="scroll"><table><tr>'+''.join(f'<th>{key}</th>' for key in rows[0])+f'</tr>{table}</table></div></section>'
    html+=f'<section><h2>LOS检查与端点采样对照</h2><p>True表示可见。端点采样只作失败示范；精确检查以障碍叶的闭矩形为准，接触也受阻。</p><div class="scroll"><table><tr><th>案例</th><th>线段</th><th>精确LOS</th><th>仅查端点</th></tr>{los_table}</table></div></section>'
    html+='<section><h2>本轮结论边界</h2><p>完成分区与基础查询验证，尚未运行A*。邻接采用遍历叶区域的几何参照，LOS采用线段与障碍叶相交检查；均不声称是作者原始实现。没有船体外形、安全距离、姿态或动力学约束。</p></section></main></html>'
    (output/'index.html').write_text(html)
    (output/'summary.md').write_text(
        '# S1运行记录\n\n合成教学地图，不是长滩港原始算例。\n\n'
        f'- 叶数：{len(leaves)}；空闲：{metrics["free_leaf_count"]}；障碍：{metrics["blocked_leaf_count"]}。\n'
        f'- 总面积：{area}；内部重叠对数：{overlaps}；邻接对称：{metrics["adjacency_symmetric"]}。\n'
        f'- 选中叶邻居中心：{metrics["selected_neighbor_centers"]}。\n'
        '- 输出：index.html、quadtree.svg、leaves.csv、queries.json、adjacency.json、metrics.json、config.json、environment.json及code_snapshot/。\n'
        '- 本脚本不自动执行单元测试；测试结果由本轮validation.json另存。\n'
        '- 不包含A*、平滑或RS；不报告规划时间、路径质量或船舶安全性。\n')
    print(json.dumps({'output_dir':str(output),'leaf_count':len(leaves),
                      'free':metrics['free_leaf_count'],'blocked':metrics['blocked_leaf_count'],
                      'area':area,'overlaps':overlaps,
                      'neighbors':metrics['selected_neighbor_centers']},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
