"""Build local-only offline A7 replay with analytic curved motion clocks."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece
from run_motion_feasibility import sample_enclosure


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in('run-dir','audit-json','prepared-dir','navigation-dir','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();x=json.loads((args.run_dir/'result.json').read_text());audit=json.loads(args.audit_json.read_text());cfg=json.loads((args.run_dir/'config.json').read_text());digest=hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest()
    if not audit['passed']or audit['source_result_sha256']!=digest:raise ValueError('Matching passed A7 audit required')
    shapes={}
    for name,key,directory in [('searchable','target_geometry_sha256',args.prepared_dir),('hazards','hazards_geometry_sha256',args.prepared_dir),('navigation','navigation_sha256',args.navigation_dir)]:
        p=directory/(name+'.geojson')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg[key]:raise ValueError('Changed replay chart')
        shapes[name]=json.loads(p.read_text())['features'][0]['geometry']
    display={name:[sample_enclosure([Piece(**p)],.02)[0]for p in body['pieces']]for name,body in x['platforms'].items()}
    payload={'platforms':x['platforms'],'scan_windows':x['scan_windows'],'end':x['mission_end_s'],'station':x['support_station'],'checks':x['distance_pairs'],
             'coverage_lower':x['coverage_lower_ratio_unchanged'],'shapes':shapes,'display_pieces':display}
    data=json.dumps(payload,ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x')as f:f.write(TEMPLATE.replace('__DATA__',data))
    args.output.with_suffix('.manifest.json').write_text(json.dumps({'result_sha256':digest,'audit_sha256':hashlib.sha256(args.audit_json.read_bytes()).hexdigest(),'builder_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'html_sha256':hashlib.sha256(args.output.read_bytes()).hexdigest(),'self_contained':True,'network_requests_required':False,'time_evaluation':'Analytic lines/arcs with exact stop-to-stop arc-length profile; not interpolation between saved CSV frames.','raw_coordinates_local_only':True},indent=2)+'\n');print(args.output)


TEMPLATE=r'''<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>A7 协同任务回放</title><link rel="icon" href="data:,">
<style>
:root{font-family:-apple-system,BlinkMacSystemFont,"PingFang SC",sans-serif;color:#19334b;background:#f3f6f8}*{box-sizing:border-box}body{margin:0;padding:22px;max-width:1440px;margin:auto}h1{font-size:25px;margin:0 0 8px}p,small{color:#5b6e7e;line-height:1.6;margin:6px 0}.layout{display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:18px;margin-top:18px}.card{background:white;border:1px solid #dce5e9;border-radius:12px;padding:18px}.mapwrap{width:100%;aspect-ratio:1}canvas{width:100%;height:100%;display:block}input[type=range]{width:100%;accent-color:#078b8b;margin:16px 0}.controls{display:flex;gap:8px;align-items:center;flex-wrap:wrap}button,select{font:inherit;padding:8px 12px;border:1px solid #cbd9df;border-radius:7px;background:#fff;color:#19334b;cursor:pointer}button.primary{background:#078b8b;color:white;border-color:#078b8b}#clock{font-size:21px;font-weight:700;font-variant-numeric:tabular-nums;margin-left:auto}h2{font-size:16px;margin:0 0 10px}.platform{border-bottom:1px solid #e6edf0;padding:12px 0;line-height:1.7;font-size:14px}.dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:7px}.metric{font-variant-numeric:tabular-nums;padding:8px 0;font-size:14px}.ok{color:#087b63}.legend{display:flex;gap:16px;flex-wrap:wrap;font-size:13px;color:#5b6e7e;margin:10px 0}footer{font-size:12px;color:#617788;line-height:1.7;margin-top:12px}.side .card{margin-bottom:14px}
@media(max-width:850px){body{padding:12px}h1{font-size:21px}.layout{grid-template-columns:1fr}.side{display:grid;grid-template-columns:1fr 1fr;gap:12px}.side .card{margin:0}#clock{width:100%;margin-left:0}.card{padding:12px}}
@media(max-width:500px){.side{grid-template-columns:1fr}}
</style>
<header><h1>A7 · 共同任务时钟与 USV 支援回放</h1><p>两艘 UUV 执行原扫描与明确补扫；USV 转场至支援驻点后驻守。拖动时间条查看扫描、转场和完成后等待。</p></header>
<div class="layout"><main class="card"><div class="mapwrap"><canvas id="map" role="img" aria-label="海图、规划曲线和三平台当前位置"></canvas></div>
<div class="legend"><span><i class="dot" style="background:#078b8b"></i>UUV1</span><span><i class="dot" style="background:#bb7726"></i>UUV2</span><span><i class="dot" style="background:#7854a2"></i>USV</span><span>实线：扫描／支援　虚线：普通转场　灰色：实体障碍</span></div>
<input id="time" type="range" min="0" value="0" step="1" aria-label="任务时间"><div class="controls"><button id="play" class="primary">播放</button><button id="reset">重置</button><button id="end">跳到结束</button><select id="speed" aria-label="回放速度"><option value="60">60×</option><option value="300">300×</option><option value="600" selected>600×</option><option value="3600">3600×</option></select><span id="clock">00:00:00</span></div>
<footer>当前位置按共同运动时钟计算。全过程约束由连续时间区间证书核查；画面用于观察任务过程。</footer></main>
<aside class="side"><section class="card"><h2>平台状态</h2><div id="state-uuv_1" class="platform"></div><div id="state-uuv_2" class="platform"></div><div id="state-usv" class="platform"></div></section>
<section class="card"><h2>当前水平距离</h2><div id="distances"></div><div id="progress" class="metric"></div><small id="coverage"></small></section>
<section class="card"><h2>全过程核查</h2><div id="bounds" class="metric ok"></div><small>要求：最小间距 25 m；USV 支援范围 3000 m。覆盖使用理想 50 m 水平圆形足迹。</small><footer>本模型包含个体曲线、启停速度和等待；未计部署／回收，也未验证真实驻守控制、水深、船体动力学或声学通信。</footer></section></aside></div>
<script>
const DATA=__DATA__, COLORS={uuv_1:'#078b8b',uuv_2:'#bb7726',usv:'#7854a2'};
const slider=document.getElementById('time'),canvas=document.getElementById('map'),ctx=canvas.getContext('2d'),background=document.createElement('canvas');
let now=0,playing=false,previous=null,W=700,pad=32,scale=1,dpr=1;slider.max=DATA.end;
function pose(piece,s){const[x,y,yaw]=piece.pose,k=piece.curvature;if(!k)return[x+s*Math.cos(yaw),y+s*Math.sin(yaw),yaw];return[x+(Math.sin(yaw+k*s)-Math.sin(yaw))/k,y+(-Math.cos(yaw+k*s)+Math.cos(yaw))/k,yaw+k*s];}
function at(body,t){const local=t-body.departure_s,ps=body.pieces,segments=body.profile.segments;let i=0,s=0,v=0,travel=0,mode='moving';if(local<0)mode='predeparture_hold';if(local>=body.profile.duration_s){i=ps.length-1;s=ps[i].length;travel=body.profile.length_m;mode='completed_hold';}else if(local>0){let lo=0,hi=segments.length-1;while(lo<hi){const m=(lo+hi)>>1;if(segments[m].t1_s>=local)hi=m;else lo=m+1;}i=lo;const p=segments[i],u=local-p.t0_s,ta=p.ramp_duration_s,tc=p.cruise_duration_s,a=p.accel_mps2,peak=p.peak_speed_mps;if(u<=ta){s=.5*a*u*u;v=a*u;}else if(u<=ta+tc){s=p.ramp_distance_m+peak*(u-ta);v=peak;}else{const r=p.t1_s-local;s=p.length_m-.5*a*r*r;v=a*r;}travel=p.s0_m+s;}const q=pose(ps[i],s);return{position_m:q.slice(0,2),heading_rad:q[2],distance_m:travel,speed_mps:v,piece_index:i,piece_distance_m:s,mode};}
function snapshot(t){const states={};for(const[n,p]of Object.entries(DATA.platforms)){states[n]=at(p,t);states[n].active_task_id=DATA.scan_windows.find(w=>w.platform===n&&w.global_start_s<=t&&t<w.global_end_s)?.task_id||'';}return{time_s:t,states,distances:{uuv_pair:Math.hypot(...states.uuv_1.position_m.map((v,i)=>v-states.uuv_2.position_m[i])),usv_uuv_1:Math.hypot(...states.usv.position_m.map((v,i)=>v-states.uuv_1.position_m[i])),usv_uuv_2:Math.hypot(...states.usv.position_m.map((v,i)=>v-states.uuv_2.position_m[i]))}};}
window.replayStateAt=snapshot;
function xy(p){return[pad+p[0]*scale,W-pad-p[1]*scale];}
function shape(c,g,color){const kind=g.type;if(kind==='GeometryCollection'){g.geometries.forEach(q=>shape(c,q,color));return;}if(kind==='MultiPolygon'){g.coordinates.forEach(q=>shape(c,{type:'Polygon',coordinates:q},color));return;}c.fillStyle=color;c.strokeStyle=color;c.lineWidth=.8;if(kind==='Polygon'){c.beginPath();for(const ring of g.coordinates){ring.forEach((p,i)=>{const q=xy(p);i?c.lineTo(...q):c.moveTo(...q)});c.closePath();}c.fill('evenodd');}else if(kind==='LineString'){c.beginPath();g.coordinates.forEach((p,i)=>i?c.lineTo(...xy(p)):c.moveTo(...xy(p)));c.stroke();}else if(kind==='MultiLineString'){g.coordinates.forEach(q=>shape(c,{type:'LineString',coordinates:q},color));}else if(kind==='Point'){c.beginPath();c.arc(...xy(g.coordinates),2,0,Math.PI*2);c.fill();}else if(kind==='MultiPoint'){g.coordinates.forEach(q=>shape(c,{type:'Point',coordinates:q},color));}}
function line(c,points,color,dashed,width,alpha=1){if(!points.length)return;c.save();c.globalAlpha=alpha;c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dashed?[4,4]:[]);c.beginPath();points.forEach((p,i)=>i?c.lineTo(...xy(p)):c.moveTo(...xy(p)));c.stroke();c.restore();}
function setup(){dpr=window.devicePixelRatio||1;W=canvas.getBoundingClientRect().width;pad=W<450?28:34;scale=(W-2*pad)/5120;canvas.width=background.width=Math.round(W*dpr);canvas.height=background.height=Math.round(W*dpr);const c=background.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,W,W);c.fillStyle='#f0f1f2';c.fillRect(pad,pad,W-2*pad,W-2*pad);c.save();c.beginPath();c.rect(pad,pad,W-2*pad,W-2*pad);c.clip();shape(c,DATA.shapes.navigation,'#d9efed');shape(c,DATA.shapes.searchable,'#dce8f3');shape(c,DATA.shapes.hazards,'#8b929a');for(const[n,pp]of Object.entries(DATA.display_pieces))for(const points of pp)line(c,points,COLORS[n],false,.8,.24);c.restore();c.strokeStyle='#9baab4';c.lineWidth=.8;c.strokeRect(pad,pad,W-2*pad,W-2*pad);c.font='11px sans-serif';c.fillStyle='#617788';c.textAlign='center';for(let k=0;k<=5000;k+=1000){const q=xy([k,0]);c.fillText(String(k/1000),q[0],W-pad+16);const p=xy([0,k]);c.textAlign='right';c.fillText(String(k/1000),pad-6,p[1]+4);c.textAlign='center';}c.fillText('东向 / km',W/2,W-3);c.save();c.translate(11,W/2);c.rotate(-Math.PI/2);c.fillText('北向 / km',0,0);c.restore();draw();}
function stamp(t){const s=Math.floor(t);return[String(Math.floor(s/3600)).padStart(2,'0'),String(Math.floor(s/60)%60).padStart(2,'0'),String(s%60).padStart(2,'0')].join(':');}
function draw(){const snap=snapshot(now);ctx.setTransform(1,0,0,1,0,0);ctx.clearRect(0,0,canvas.width,canvas.height);ctx.drawImage(background,0,0);ctx.setTransform(dpr,0,0,dpr,0,0);ctx.save();ctx.beginPath();ctx.rect(pad,pad,W-2*pad,W-2*pad);ctx.clip();for(const[n,p]of Object.entries(DATA.platforms)){const state=snap.states[n];for(let i=0;i<=state.piece_index;i++){const segment=p.profile.segments[i],mid=p.departure_s+(segment.t0_s+segment.t1_s)/2,active=DATA.scan_windows.some(w=>w.platform===n&&w.global_start_s<=mid&&mid<w.global_end_s);let points=DATA.display_pieces[n][i];if(i===state.piece_index){const piece=p.pieces[i],fraction=state.piece_distance_m/piece.length,steps=Math.max(1,Math.ceil(points.length*fraction));points=Array.from({length:steps+1},(_,j)=>pose(piece,state.piece_distance_m*j/steps).slice(0,2));}line(ctx,points,COLORS[n],n!=='usv'&&!active,active||n==='usv'?1.6:1);}const q=xy(state.position_m);if(state.active_task_id){ctx.fillStyle=COLORS[n]+'22';ctx.beginPath();ctx.arc(...q,50*scale,0,Math.PI*2);ctx.fill();}ctx.save();ctx.translate(...q);ctx.rotate(-state.heading_rad);ctx.fillStyle=COLORS[n];ctx.strokeStyle='white';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(8,0);ctx.lineTo(-6,-5);ctx.lineTo(-3,0);ctx.lineTo(-6,5);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();let label=state.active_task_id?state.active_task_id.startsWith('repair_')?'明确补扫':'原任务扫描':state.mode==='completed_hold'?(n==='usv'?'驻守支援':'完成后等待'):state.mode==='predeparture_hold'?'出发前等待':'转场';document.getElementById('state-'+n).innerHTML=`<b><i class="dot" style="background:${COLORS[n]}"></i>${n.toUpperCase().replace('_','')}</b> · ${label}<br><span style="color:#617788">${state.active_task_id||'扫描关闭'} · ${state.speed_mps.toFixed(2)} m/s</span>`;}
ctx.restore();document.getElementById('clock').textContent=stamp(now)+' / '+stamp(DATA.end);slider.value=now;document.getElementById('distances').innerHTML=`<div class="metric">UUV1 — UUV2：<b>${snap.distances.uuv_pair.toFixed(1)} m</b></div><div class="metric">USV — UUV1：<b>${snap.distances.usv_uuv_1.toFixed(1)} m</b></div><div class="metric">USV — UUV2：<b>${snap.distances.usv_uuv_2.toFixed(1)} m</b></div>`;const done=DATA.scan_windows.filter(w=>w.global_end_s<=now);document.getElementById('progress').textContent=`已完成：原扫描 ${done.filter(w=>w.kind==='source_scan').length}/86；补扫 ${done.filter(w=>w.kind==='repair_scan').length}/3`;
}
const minBound=Math.min(...Object.values(DATA.checks).map(v=>v.minimum_distance_lower_m)),maxBound=Math.max(DATA.checks.usv__uuv_1.maximum_distance_upper_m,DATA.checks.usv__uuv_2.maximum_distance_upper_m);document.getElementById('bounds').textContent=`最小间距 ≥ ${(Math.floor(minBound*100)/100).toFixed(2)} m；最大支援距离 ≤ ${(Math.ceil(maxBound*100)/100).toFixed(2)} m`;document.getElementById('coverage').textContent=`计划最终覆盖下界 ${(DATA.coverage_lower*100).toFixed(4)}%（此处不计算实时覆盖面积）`;
function pause(){playing=false;previous=null;document.getElementById('play').textContent='播放';}
slider.addEventListener('input',()=>{pause();now=Number(slider.value);draw()});document.getElementById('play').onclick=()=>{if(playing){pause();return}if(now>=DATA.end)now=0;playing=true;previous=null;document.getElementById('play').textContent='暂停';requestAnimationFrame(tick)};document.getElementById('reset').onclick=()=>{pause();now=0;draw()};document.getElementById('end').onclick=()=>{pause();now=DATA.end;draw()};function tick(ts){if(!playing)return;if(previous!==null)now=Math.min(DATA.end,now+(ts-previous)/1000*Number(document.getElementById('speed').value));previous=ts;draw();if(now>=DATA.end)pause();else requestAnimationFrame(tick)}window.addEventListener('resize',setup);setup();
</script></html>'''


if __name__=='__main__':main()
