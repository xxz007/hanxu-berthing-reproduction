"""Build an offline time replay from a saved and independently audited A3 run.

The HTML contains derived chart/route coordinates and stays in ignored outputs.
The committed generator and preview figure are sufficient for repository reuse.
"""
import argparse
import hashlib
import json
from pathlib import Path

from chart_geometry import Geos


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir', type=Path, required=True)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--audit-json', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = json.loads((args.run_dir/'result.json').read_text())
    provenance = json.loads((args.run_dir/'provenance.json').read_text())
    audit = json.loads(args.audit_json.read_text())
    if result['status'] != 'cooperative_geometry_passed' or not audit['passed']:
        raise ValueError('Replay requires a passed independent audit')
    if hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest() != provenance['result_sha256']:
        raise ValueError('Saved A3 result changed')
    target_path = args.prepared_dir/'searchable.geojson'
    if hashlib.sha256(target_path.read_bytes()).hexdigest() != result['target_geometry_sha256']:
        raise ValueError('Replay target differs from planned target')
    manifest = json.loads((args.prepared_dir/'manifest.json').read_text())
    shapes = {}
    viewport = {'type': 'Polygon', 'coordinates': [[[500,700],[5250,700],[5250,4400],[500,4400],[500,700]]]}
    with Geos() as geos:
        for name in ('land', 'excluded_task', 'searchable', 'task_boundary'):
            path = args.prepared_dir/(name+'.geojson')
            if hashlib.sha256(path.read_bytes()).hexdigest() != manifest['derived_files_sha256'][path.name]:
                raise ValueError('Changed chart drawing input: '+name)
            shape = json.loads(path.read_text())['features'][0]['geometry']
            if name == 'task_boundary' and not geos.covers(viewport, shape):
                raise ValueError('Display viewport hides task region')
            shapes[name] = geos.operation('intersection', shape, viewport)
    payload = {'platforms': result['platforms'], 'end': result['mission_end_s'], 'shapes': shapes,
               'config': json.loads((args.run_dir/'config.json').read_text()),
               'coverage': result['coverage'], 'checks': result['pairwise_distances']}
    encoded = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).replace('</', '<\\/')
    html = TEMPLATE.replace('__DATA__', encoded)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        stream.write(html)
    meta = {'html_sha256': hashlib.sha256(args.output.read_bytes()).hexdigest(),
            'result_sha256': provenance['result_sha256'],
            'audit_sha256': hashlib.sha256(args.audit_json.read_bytes()).hexdigest(),
            'self_contained': True, 'network_requests_required': False,
            'time_interpolation': 'piecewise linear saved timeline; no smoothing/easing',
            'raw_coordinate_output_local_only': True}
    args.output.with_suffix('.manifest.json').write_text(json.dumps(meta, indent=2)+'\n')
    print(json.dumps({'output': str(args.output), 'self_contained': True}))


TEMPLATE = r'''<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>A3 · USV / UUV 协同覆盖回放</title><link rel="icon" href="data:,">
<style>
:root{font-family:-apple-system,BlinkMacSystemFont,"PingFang SC",sans-serif;color:#19334b;background:#f3f6f7}
*{box-sizing:border-box}body{margin:0;padding:24px;max-width:1540px;margin-inline:auto}h1{font-size:25px;margin:0 0 8px}p{line-height:1.65;margin:8px 0;color:#536775}header{margin-bottom:20px}
.layout{display:grid;grid-template-columns:minmax(500px,1fr) 310px;gap:18px}.panel{background:white;border:1px solid #dce4e6;border-radius:12px;padding:18px}.mapwrap{position:relative}canvas{display:block;width:100%;height:auto;background:#f4fafc;border-radius:5px}
.legend{display:flex;gap:16px;flex-wrap:wrap;font-size:13px;padding:10px 0}.swatch{display:inline-block;width:18px;height:3px;vertical-align:middle;margin-right:6px}.card{border-left:4px solid var(--c);padding:8px 12px;background:#f7f9fa;margin-bottom:12px;border-radius:4px}.card strong{display:block;margin-bottom:6px}.card span{font-size:14px;line-height:1.65;display:block}
.controls{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin-top:12px}button,select{border:1px solid #bccbd0;background:white;color:#19334b;border-radius:6px;padding:8px 12px;cursor:pointer}button.primary{background:#19334b;color:white}input[type=range]{width:100%;margin:16px 0;accent-color:#078b8b}#clock{font-variant-numeric:tabular-nums;font-weight:700}h2{font-size:17px;margin:6px 0 12px}.value{font-size:14px;border-top:1px solid #e5ebed;padding:10px 0}.value strong{float:right;font-variant-numeric:tabular-nums}small{line-height:1.7;display:block;color:#5b6c78}footer{margin-top:16px;font-size:13px;color:#5b6c78;line-height:1.7}
@media(max-width:900px){body{padding:12px}.layout{grid-template-columns:1fr}.side{display:grid;grid-template-columns:1fr 1fr;gap:10px}.side>section{grid-column:1/-1}}
</style>
<header><h1>真实海图上的 USV / UUV 协同覆盖</h1><p>两艘 UUV 分担原有扫描任务，一艘 USV 提供几何支援。拖动时间条查看分工、等待和平台间距。</p></header>
<div class="layout"><main class="panel"><div class="mapwrap"><canvas id="map" width="1140" height="900" aria-label="海图与三平台时间回放"></canvas></div>
<div class="legend"><span><i class="swatch" style="background:#078b8b"></i>UUV 1</span><span><i class="swatch" style="background:#b36b16"></i>UUV 2</span><span><i class="swatch" style="background:#744c99"></i>USV 支援</span><span>实线：扫描　虚线：连接　灰色：陆地／障碍背景</span></div>
<input id="time" type="range" min="0" max="1000" value="0" step="1" aria-label="任务时间">
<div class="controls"><button id="play" class="primary">播放</button><button id="reset">回到起点</button><label>回放速度 <select id="speed"><option value="200">200 倍</option><option value="1000" selected>1000 倍</option><option value="3000">3000 倍</option></select></label><span id="clock"></span><label><input id="future" type="checkbox" checked>显示完整路线</label></div>
</main><aside class="panel side"><div class="card" style="--c:#078b8b"><strong>UUV 1 · 南侧任务带</strong><span id="uuv_1"></span></div><div class="card" style="--c:#b36b16"><strong>UUV 2 · 北侧任务带</strong><span id="uuv_2"></span></div><div class="card" style="--c:#744c99"><strong>USV · 水面支援</strong><span id="usv"></span></div>
<section><h2>当前水平距离</h2><div class="value">两艘 UUV<strong id="d12"></strong></div><div class="value">USV ↔ UUV 1<strong id="d01"></strong></div><div class="value">USV ↔ UUV 2<strong id="d02"></strong></div><small>阈值：所有平台间距 ≥25 m；USV 到每艘 UUV ≤3000 m。全时钟验收采用分段解析计算，动画显示仅用于观察。</small></section>
<section><h2>最终覆盖</h2><div class="value">原完整目标<strong id="area"></strong></div><div class="value">覆盖率<strong id="coverage"></strong></div><div class="value">剩余漏扫<strong id="miss"></strong></div><small>以上为任务全部完成后的固定足迹结果，未将转场或 USV 计入扫描。</small></section></aside></div>
<footer>NOAA 长滩 ENC 背景，沿用 A1 / A2 输入。三平台已部署到初始位置后开始计时，结束后原地等待；不包含出航、下潜、回收和返港。航速、水平间距和支援距离均为本次工程设定。本回放是二维点平台几何时钟，不验证转弯、动力学、实际声学通信和水下净空。</footer>
<script>
'use strict';const DATA=__DATA__,COLORS={uuv_1:'#078b8b',uuv_2:'#b36b16',usv:'#744c99'};
const canvas=document.getElementById('map'),ctx=canvas.getContext('2d'),slider=document.getElementById('time');
let now=0,playing=false,last=0;const bounds=[500,700,5250,4400],pad=55;
const scale=Math.min((canvas.width-pad*2)/(bounds[2]-bounds[0]),(canvas.height-pad*2)/(bounds[3]-bounds[1]));
const xy=p=>[pad+(p[0]-bounds[0])*scale,canvas.height-pad-(p[1]-bounds[1])*scale];
function shape(g,fill,stroke){if(g.type==='GeometryCollection'){g.geometries.forEach(x=>shape(x,fill,stroke));return;}const polys=g.type==='Polygon'?[g.coordinates]:g.type==='MultiPolygon'?g.coordinates:[];for(const poly of polys){ctx.beginPath();for(const ring of poly){ring.forEach((p,i)=>{const q=xy(p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.closePath()}if(fill){ctx.fillStyle=fill;ctx.fill('evenodd')}if(stroke){ctx.strokeStyle=stroke;ctx.lineWidth=1;ctx.stroke()}}}
function at(platform,t){const tl=platform.timeline,ss=tl.segments;for(const s of ss){if(t<=s.t1_s){const f=s.t1_s===s.t0_s?1:Math.max(0,(t-s.t0_s)/(s.t1_s-s.t0_s));return{p:s.p0.map((x,i)=>x+(s.p1[i]-x)*f),s}}}return{p:tl.final_position,s:ss[ss.length-1]};}
function line(points,color,dash=[],alpha=1,width=2){ctx.beginPath();points.forEach((p,i)=>{const q=xy(p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.strokeStyle=color;ctx.lineWidth=width;ctx.globalAlpha=alpha;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);ctx.globalAlpha=1;}
function hours(t){const h=Math.floor(t/3600),m=Math.floor(t%3600/60),s=Math.floor(t%60);return[h,m,s].map(x=>String(x).padStart(2,'0')).join(':');}
function draw(){ctx.clearRect(0,0,canvas.width,canvas.height);shape(DATA.shapes.land,'#b8b7ac');shape(DATA.shapes.excluded_task,'#f1e7dc');shape(DATA.shapes.searchable,'#e5f2f1');shape(DATA.shapes.task_boundary,null,'#657b8c');
ctx.font='16px -apple-system,sans-serif';ctx.fillStyle='#536775';ctx.strokeStyle='#d8e5e8';ctx.lineWidth=.7;for(let x=1000;x<=5000;x+=1000){const p=xy([x,bounds[1]]);ctx.beginPath();ctx.moveTo(p[0],pad);ctx.lineTo(p[0],canvas.height-pad);ctx.stroke();ctx.fillText((x/1000).toFixed(0),p[0]-5,canvas.height-pad+25)}for(let y=1000;y<=4000;y+=1000){const p=xy([bounds[0],y]);ctx.beginPath();ctx.moveTo(pad,p[1]);ctx.lineTo(canvas.width-pad,p[1]);ctx.stroke();ctx.fillText((y/1000).toFixed(0),pad-25,p[1]+5)}ctx.fillText('东向 / km',canvas.width/2-30,canvas.height-10);ctx.save();ctx.translate(17,canvas.height/2+30);ctx.rotate(-Math.PI/2);ctx.fillText('北向 / km',0,0);ctx.restore();
const positions={};for(const [id,p]of Object.entries(DATA.platforms)){const color=COLORS[id];if(document.getElementById('future').checked)for(const leg of p.legs)line(leg.points,color,leg.scanning?[]:[6,6],.19,1.7);for(const s of p.timeline.segments){if(s.t0_s>now)break;if(s.p0[0]===s.p1[0]&&s.p0[1]===s.p1[1])continue;const f=Math.min(1,(now-s.t0_s)/(s.t1_s-s.t0_s));line([s.p0,s.p0.map((x,i)=>x+(s.p1[i]-x)*f)],color,s.scanning?[]:[6,5],1,s.scanning?2.8:2)}const state=at(p,now);positions[id]=state.p;const q=xy(state.p);ctx.save();ctx.translate(...q);const a=xy(state.s.p0),b=xy(state.s.p1);ctx.rotate(Math.atan2(b[1]-a[1],b[0]-a[0]));ctx.beginPath();ctx.moveTo(12,0);ctx.lineTo(-9,-7);ctx.lineTo(-5,0);ctx.lineTo(-9,7);ctx.closePath();ctx.fillStyle=color;ctx.fill();ctx.strokeStyle='white';ctx.lineWidth=2;ctx.stroke();ctx.restore();const complete=p.timeline.segments.filter(s=>s.scanning&&s.t1_s<=now).length;const moving=Math.hypot(state.s.p1[0]-state.s.p0[0],state.s.p1[1]-state.s.p0[1])>0;const finished=id==='usv'?now>=DATA.end:now>=p.timeline.movement_finish_time_s;document.getElementById(id).innerHTML=(id==='usv'?(finished?'支援完成':moving?'支援转场':'驻点支援'):`${finished?'任务完成':state.s.scanning?'扫描':moving?'避障连接':'等待'} · 完成 ${complete} / ${p.task_ids.length} 条`)+`<br>位置 ${(state.p[0]/1000).toFixed(2)}, ${(state.p[1]/1000).toFixed(2)} km<br>路线 ${(p.length_m/1000).toFixed(2)} km`;}
for(const[id,a,b]of[['d12','uuv_1','uuv_2'],['d01','usv','uuv_1'],['d02','usv','uuv_2']])document.getElementById(id).textContent=Math.hypot(positions[a][0]-positions[b][0],positions[a][1]-positions[b][1]).toFixed(1)+' m';document.getElementById('clock').textContent=hours(now)+' / '+hours(DATA.end);slider.value=1000*now/DATA.end;}
document.getElementById('area').textContent=(DATA.coverage.target_area_m2/1e6).toFixed(3)+' km²';document.getElementById('coverage').textContent=(DATA.coverage.coverage_ratio*100).toFixed(3)+'%';document.getElementById('miss').textContent=DATA.coverage.missed_area_m2.toFixed(0)+' m²';document.getElementById('play').onclick=()=>{if(now>=DATA.end)now=0;playing=!playing;document.getElementById('play').textContent=playing?'暂停':'播放';};document.getElementById('reset').onclick=()=>{playing=false;now=0;document.getElementById('play').textContent='播放';draw()};slider.oninput=()=>{now=Number(slider.value)/1000*DATA.end;draw()};document.getElementById('future').onchange=draw;
function frame(t){if(playing&&last){now=Math.min(DATA.end,now+(t-last)/1000*Number(document.getElementById('speed').value));if(now>=DATA.end){playing=false;document.getElementById('play').textContent='播放'}draw()}last=t;requestAnimationFrame(frame)}draw();requestAnimationFrame(frame);
</script></html>'''


if __name__ == '__main__':
    main()
