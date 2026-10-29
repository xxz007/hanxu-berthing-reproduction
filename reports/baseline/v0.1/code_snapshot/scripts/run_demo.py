"""Run the full Chapter 3 learning pipeline; stdlib only, never overwrite runs."""
import argparse
import csv
from dataclasses import asdict
from datetime import datetime,timezone
import hashlib
from html import escape
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.pipeline import run,validate_config


def map_svg(config,tree,lines,show_cells=False,cusps=(),ship=False,route_groups=()):
    bounds = config['map']['bounds']
    x0,y0,x1,y1 = bounds
    scale = 440/max(x1-x0,y1-y0)
    def xy(p):
        return 55+(p[0]-x0)*scale,485-(p[1]-y0)*scale
    out = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 530 545" role="img" aria-label="合成港口路径，米制坐标"><rect width="530" height="545" fill="#fff"/>',
           '<rect x="55" y="45" width="440" height="440" fill="#edf6fa"/>']
    if show_cells:
        for cell in tree.leaves.values():
            a,b,c,d = cell.bounds
            x,y = xy((a,d))
            fill = '#d7dfe3' if cell.blocked else 'none'
            out.append(f'<rect x="{x:.3f}" y="{y:.3f}" width="{(c-a)*scale:.3f}" height="{(d-b)*scale:.3f}" fill="{fill}" stroke="#b8c9d2" stroke-width=".32"/>')
    for a,b,c,d in config['map']['obstacles']:
        x,y = xy((a,d))
        out.append(f'<rect x="{x:.3f}" y="{y:.3f}" width="{(c-a)*scale:.3f}" height="{(d-b)*scale:.3f}" fill="#526271"/>')
    for points,color,width,dash in lines:
        path = ' '.join(f'{xy(p)[0]:.3f},{xy(p)[1]:.3f}' for p in points)
        out.append(f'<polyline points="{path}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" stroke-dasharray="{dash}"/>')
    for p in cusps:
        x,y = xy(p)
        out.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="5" fill="#fff" stroke="#b4455b" stroke-width="2"/>')
    for name,group_lines,group_cusps in route_groups:
        out.append(f'<g data-route="{escape(name)}">')
        for points,color,width,dash in group_lines:
            path = ' '.join(f'{xy(p)[0]:.3f},{xy(p)[1]:.3f}' for p in points)
            out.append(f'<polyline points="{path}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" stroke-dasharray="{dash}"/>')
        for p in group_cusps:
            x,y = xy(p)
            out.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="5" fill="#fff" stroke="#b4455b" stroke-width="2"/>')
        out.append('</g>')
    for s in config['scenarios']:
        p = s['goal']
        x,y = xy(p)
        out.append(f'<path d="M{x-4},{y-4}l8,8m0,-8l-8,8" stroke="#344454" stroke-width="2"/>')
    x,y = xy(config['scenarios'][0]['start'])
    out.append(f'<circle cx="{x}" cy="{y}" r="4" fill="#187d77"/>')
    for i in range(5):
        tx,ty = x0+i*(x1-x0)/4,y0+i*(y1-y0)/4
        x,y = xy((tx,ty))
        out.append(f'<text x="{x}" y="505" text-anchor="middle" font-size="12" fill="#4a5c6c">{tx:g}</text><text x="46" y="{y+4}" text-anchor="end" font-size="12" fill="#4a5c6c">{ty:g}</text>')
    out.append('<text x="275" y="530" text-anchor="middle" font-size="13">x / m</text><text x="14" y="265" text-anchor="middle" font-size="13" transform="rotate(-90,14,265)">y / m</text>')
    if ship:
        out.append('<g id="boat"><path d="M12,0 L-7,-5 L-11,0 L-7,5 Z" fill="#222f43" stroke="white" stroke-width="1.3"/></g>')
    return ''.join(out)+'</svg>'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/harbor_demo.json')
    parser.add_argument('--output-dir',type=Path)
    args = parser.parse_args()
    raw = args.config.read_bytes()
    config = json.loads(raw)
    validate_config(config)
    output = args.output_dir or ROOT/'outputs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_full_demo')
    if output.exists():
        raise FileExistsError(f'Output exists; preserving previous results: {output}')
    tree,grid,map_seconds,results = run(config)
    output.mkdir(parents=True,exist_ok=False)
    (output/'config.json').write_bytes(raw)
    metrics = dict(data_kind=config['data_kind'],map_seconds=map_seconds,
                   quadtree_leaves=len(tree.leaves),quadtree_free=sum(not c.blocked for c in tree.leaves.values()),
                   uniform_cells=len(grid.leaves),uniform_free=sum(not c.blocked for c in grid.leaves.values()),scenarios={})
    panels = []
    first = results[0]
    stage_specs = [('01_uniform_astar','均匀网格 A*',grid,first['baseline']['path'],True,'#75859a'),
                   ('02_multiscale_astar','多尺度 A*',tree,first['multi']['path'],True,'#bf8153'),
                   ('03_los_parent','LOS 父节点候选改进',tree,first['improved']['path'],True,'#b4455b'),
                   ('04_g2cbs','G2CBS 基本转角平滑',tree,first['smooth'],False,'#3d7396')]
    for filename,title,partition,path,cells,color in stage_specs:
        svg = map_svg(config,partition,[(path,color,2.4,'')],cells)
        (output/(filename+'.svg')).write_text(svg)
        panels.append(f'<article><h3>{title}</h3>{svg}</article>')
    table,route_groups = [],[]
    for r in results:
        name = r['name']
        metrics['scenarios'][name] = dict(r['metrics'])
        for method in ('baseline','multi','improved'):
            stats = {k:v for k,v in r[method].items() if k not in ('path','closed_points')}
            metrics['scenarios'][name][method] = stats
        with (output/(name+'_trajectory.csv')).open('w',newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x_m','y_m','yaw_rad','curvature_per_m','direction'])
            writer.writerows(r['samples'])
        (output/(name+'_primitives.json')).write_text(json.dumps([asdict(p) for p in r['pieces']],indent=2)+'\n')
        (output/(name+'_stages.json')).write_text(json.dumps({k:r[k] for k in ('baseline','multi','improved','reduced','smooth','cusps')},indent=2)+'\n')
        forward,reverse = [],[]
        for p in r['pieces']:
            (forward if p.direction==1 else reverse).append(p.samples(config['sample_step_m']))
        lines = [(path,'#187d77',2.6,'') for path in forward]+[(path,'#b4455b',3,'6 3') for path in reverse]
        route_groups.append((name,lines,r['cusps']))
        svg = map_svg(config,tree,lines,cusps=r['cusps'])
        (output/(name+'.svg')).write_text(svg)
        title = '通航算例：完整路径' if name=='transit' else '船艉抵泊：完整路径'
        panels.append(f'<article><h3>{title}</h3>{svg}</article>')
        m = r['metrics']
        table.append(f'<tr><td>{title}</td><td>{m["length_m"]:.1f}</td><td>{m["reverse_m"]:.1f}</td><td>{m["cusps"]}</td><td>{m["maximum_curvature_extrema"]:.6f}</td><td>通过</td></tr>')
    (output/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')
    code = list((ROOT/'src').rglob('*.py'))+[ROOT/'scripts/run_demo.py']
    hashes = {}
    for path in code:
        relative = path.relative_to(ROOT)
        dest = output/'code_snapshot'/relative
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,dest)
        hashes[str(relative)] = hashlib.sha256(path.read_bytes()).hexdigest()
    git = subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,capture_output=True,text=True)
    status = subprocess.run(['git','status','--porcelain'],cwd=ROOT,capture_output=True,text=True)
    env = dict(python=platform.python_version(),platform=platform.platform(),third_party_dependencies=[],
               configuration_sha256=hashlib.sha256(raw).hexdigest(),source_sha256=hashes,
               git_commit=git.stdout.strip() if git.returncode==0 else None,git_dirty=bool(status.stdout),
               created_utc=datetime.now(timezone.utc).isoformat())
    (output/'environment.json').write_text(json.dumps(env,indent=2)+'\n')
    overview = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1620 1260"><rect width="1620" height="1260" fill="white"/><g font-family="system-ui,sans-serif" fill="#233443"><text x="35" y="44" font-size="30">Chapter 3 · Overall learning reproduction</text><text x="35" y="76" font-size="18">Synthetic harbor | Green: forward | Dashed red: reverse | White circle: direction switch | Rmin = 200 m</text>']
    names = ['01_uniform_astar','02_multiscale_astar','03_los_parent','04_g2cbs','transit','stern_berthing']
    titles = ['A · Uniform-grid A*','B · Multiscale A*','C · LOS-parent relaxation','D · G2CBS basic corner pair','E · Full transit path','F · Stern berthing path']
    for i,(name,title) in enumerate(zip(names,titles)):
        x,y = 10+(i%3)*535,125+(i//3)*575
        overview.append(f'<text x="{x+20}" y="{y-10}" font-size="23">{title}</text>')
        nested = (output/(name+'.svg')).read_text().replace('<svg ',f'<svg x="{x}" y="{y}" width="520" height="535" ',1)
        overview.append(nested)
    (output/'overview.svg').write_text(''.join(overview)+'</g></svg>')
    # Standard-library SVG/HTML learning report. Animation is geometric playback.
    replay_data = {r['name']:r['samples'] for r in results}
    animation_map = map_svg(config,tree,[],ship=True,route_groups=route_groups)
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>第三章 · 整体复现实验</title>
<style>body{margin:0;background:#f3f6f8;color:#233443;font:16px/1.65 system-ui,sans-serif}main{max-width:1180px;margin:auto;padding:32px}h1{font-size:30px;letter-spacing:-.5px;margin-bottom:4px}h2{font-size:22px;margin-top:32px}h3{font-size:17px;margin:0;padding:18px 20px 0}p{max-width:900px}.tag{color:#187d77;font-size:13px;letter-spacing:2px}.notice{background:#fff5df;border-left:4px solid #ba8444;padding:14px 18px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}article,.replay{background:white;border-radius:12px;overflow:hidden;border:1px solid #dce4e9}svg{display:block;width:100%;height:auto}.replay{display:grid;grid-template-columns:minmax(300px,560px) 1fr;align-items:center}.controls{padding:25px}input{width:100%}button,select{font:inherit;padding:8px;border:1px solid #b6c4ce;border-radius:6px;background:white;margin:4px 8px 8px 0}table{width:100%;border-collapse:collapse;background:white}th,td{text-align:left;padding:10px;border-bottom:1px solid #dce4e9;font-size:14px}a{color:#187d77}.small{font-size:14px;color:#536774}@media(max-width:800px){.grid{grid-template-columns:repeat(2,1fr)}.replay{display:block}main{padding:18px}.scroll{overflow:auto}}@media(max-width:480px){.grid{grid-template-columns:1fr}h1{font-size:25px}}</style>
<main><div class="tag">HAN XU 2023 · CHAPTER 3 · LEARNING REPRODUCTION</div><h1>先看全貌，再逐块攻坚</h1><p>合成港口上的多尺度 A*、G2CBS 与 Reeds–Shepp 靠泊规划。所有图和数值由本次程序实际生成。</p>
<div class="notice"><b>这是整体流程复现，尚未完成论文算例数值复现。</b>地图为矩形港口替代图；父节点更新采用明确的 Theta* 风格候选规则；G2CBS 为基本双三次曲线转角构造，缺少短航段合并；RS 处允许曲率跳变。仅检查带 30 m 余量的中心线，未模拟船体、速度、MPC 或真实航行。</div>
<h2>路径回放</h2><div class="replay">ANIMATION<div class="controls"><label for="scenario">算例</label><select id="scenario"><option value="transit">通航到达</option><option value="stern_berthing">船艉抵泊</option></select><br><button id="play">播放</button><button id="reset">重置</button><label for="progress">路径进度</label><input type="range" id="progress" min="0" max="1000" value="0"><p id="readout"></p><p class="small">绿色实线：前进；红色虚线：倒车；白圈：方向切换点。船形朝向表示船首朝向，倒车时运动方向相反。播放速度没有物理时间含义，切换点仅标示应设计零速的位置。</p></div></div>
<h2>各阶段对照</h2><div class="grid">PANELS</div><h2>本次计算结果</h2><div class="scroll"><table><thead><tr><th>算例</th><th>路径长 / m</th><th>倒车 / m</th><th>切换数</th><th>曲率上界 / m⁻¹</th><th>几何检查</th></tr></thead><tbody>TABLE</tbody></table></div>
<p class="small">路径长度按 ≤5 m 采样折线近似；倒车长度由 RS 基元精确相加。曲率按解析曲线及数值极值检查，浮点检查并非形式化证明。RS 与平滑段连接处位置、船首朝向连续，曲率可不连续。纸面表3.3数值未混入本表。</p>
<p><a href="metrics.json">指标与衔接检查</a> · <a href="config.json">实际配置</a> · <a href="environment.json">环境与代码哈希</a> · <a href="transit_trajectory.csv">通航轨迹</a> · <a href="stern_berthing_trajectory.csv">船艉抵泊轨迹</a></p></main>
<script>const DATA=REPLAYDATA, BOUNDS=MAPBOUNDS;const slider=document.getElementById('progress'),choice=document.getElementById('scenario'),boat=document.getElementById('boat');let playing=false,last=0;function draw(){document.querySelectorAll('[data-route]').forEach(g=>{g.style.display=g.dataset.route===choice.value?'':'none'});const scale=440/Math.max(BOUNDS[2]-BOUNDS[0],BOUNDS[3]-BOUNDS[1]);const p=DATA[choice.value][Math.round(Number(slider.value)/1000*(DATA[choice.value].length-1))];boat.setAttribute('transform',`translate(${55+(p[0]-BOUNDS[0])*scale},${485-(p[1]-BOUNDS[1])*scale}) rotate(${-p[2]*180/Math.PI})`);document.getElementById('readout').textContent=`${p[4]>0?'前进':'倒车'} · x=${p[0].toFixed(1)} m · y=${p[1].toFixed(1)} m · 船首角=${(p[2]*180/Math.PI).toFixed(1)}°`;}slider.addEventListener('input',draw);choice.addEventListener('change',()=>{slider.value=0;draw()});document.getElementById('reset').onclick=()=>{playing=false;slider.value=0;draw();document.getElementById('play').textContent='播放'};document.getElementById('play').onclick=()=>{playing=!playing;document.getElementById('play').textContent=playing?'暂停':'播放'};function frame(t){if(playing&&t-last>25){slider.value=Math.min(1000,Number(slider.value)+2);last=t;draw();if(Number(slider.value)===1000){playing=false;document.getElementById('play').textContent='播放'}}requestAnimationFrame(frame)}draw();requestAnimationFrame(frame);</script></html>'''
    page = page.replace('ANIMATION',animation_map).replace('PANELS',''.join(panels)).replace('TABLE',''.join(table)).replace('REPLAYDATA',json.dumps(replay_data)).replace('MAPBOUNDS',json.dumps(config['map']['bounds']))
    (output/'index.html').write_text(page)
    print(output)
    print(json.dumps({r['name']:{k:v for k,v in r['metrics'].items() if k in ('validation_passed','length_m','reverse_m','maximum_curvature_extrema','cusps')} for r in results},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
