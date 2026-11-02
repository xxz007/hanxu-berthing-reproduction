"""Plot all fixed-pose A5 connector outcomes after independent audit."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('prepared-dir','navigation-dir','trial-dir','audit-json','output-dir','qa-script-dir'):
        parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args();audit=json.loads(args.audit_json.read_text());trial=json.loads((args.trial_dir/'result.json').read_text())
    if not audit['passed']:raise ValueError('A5 figure requires passed experiment audit')
    provenance=json.loads((args.trial_dir/'provenance.json').read_text());cfg=json.loads((args.trial_dir/'config.json').read_text())
    result_hash=hashlib.sha256((args.trial_dir/'result.json').read_bytes()).hexdigest()
    if result_hash!=provenance['result_sha256']or result_hash!=audit['source_result_sha256']:raise ValueError('Changed plotted result')
    if audit['connector_counts']!=trial['connector_counts']:raise ValueError('Audit/result count mismatch')
    shapes={}
    for name in ('searchable','hazards'):
        p=args.prepared_dir/(name+'.geojson')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg['A1_geometry_sha256'][name]:raise ValueError('Changed drawing input')
        shapes[name]=json.loads(p.read_text())['features'][0]['geometry']
    p=args.navigation_dir/'navigation.geojson'
    if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg['expected_navigation_sha256']:raise ValueError('Changed navigation')
    shapes['navigation']=json.loads(p.read_text())['features'][0]['geometry']
    args.output_dir.mkdir(parents=True,exist_ok=False)
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.path import Path as MplPath
    from matplotlib.patches import PathPatch,Patch
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Noto Sans CJK SC','Arial','DejaVu Sans'],
                         'font.size':10,'axes.unicode_minus':False,'pdf.fonttype':42,'svg.fonttype':'none',
                         'figure.facecolor':'white','savefig.facecolor':'white'})
    ink='#19334b';green='#078b8b';orange='#c46c2c'
    def draw(ax,shape,color):
        kind=shape['type']
        if kind=='GeometryCollection':
            for member in shape['geometries']:draw(ax,member,color)
        elif kind=='MultiPolygon':
            for poly in shape['coordinates']:draw(ax,{'type':'Polygon','coordinates':poly},color)
        elif kind=='Polygon':
            vertices=[];codes=[]
            for i,ring in enumerate(shape['coordinates']):
                if not ring:continue
                xy=np.asarray(ring);signed=np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])
                if(signed>0)!=(i==0):xy=xy[::-1]
                vertices.extend(xy);codes.extend([MplPath.MOVETO]+[MplPath.LINETO]*(len(xy)-2)+[MplPath.CLOSEPOLY])
            if vertices:ax.add_patch(PathPatch(MplPath(vertices,codes),facecolor=color,edgecolor='none'))
        elif kind in ('LineString','MultiLineString'):
            lines=[shape['coordinates']]if kind=='LineString'else shape['coordinates']
            for line in lines:
                xy=np.asarray(line)
                if len(xy):ax.plot(xy[:,0],xy[:,1],color=color,lw=.7)
        elif kind in ('Point','MultiPoint'):
            xy=np.asarray([shape['coordinates']]if kind=='Point'else shape['coordinates'])
            if len(xy):ax.scatter(xy[:,0],xy[:,1],color=color,s=4)
    totals={label:sum(v['passed']for v in trial['connector_counts'][label].values())for label in ('original_domain','separated_domain')}
    required=len(trial['connections'])
    fig,axes=plt.subplots(1,2,figsize=(13,8.6))
    fig.subplots_adjust(left=.065,right=.98,bottom=.24,top=.81,wspace=.17)
    for letter,ax,label,title in zip('ab',axes,('original_domain','separated_domain'),('任务区内导航','独立安全导航')):
        ax.set_facecolor('#f0f1f2')
        if label=='separated_domain':draw(ax,shapes['navigation'],'#cbe9e6')
        draw(ax,shapes['searchable'],'#dce8f3');draw(ax,shapes['hazards'],'#8b929a')
        failed=[]
        for r in trial['connections']:
            if r[label]['passed']:
                xy=np.asarray(r[label]['certificate']['points'])
                if len(xy):ax.plot(xy[:,0],xy[:,1],color=green,lw=.65,alpha=.9)
            else:failed.extend([r['start_pose'][:2],r['goal_pose'][:2]])
        if failed:
            xy=np.asarray(failed);ax.scatter(xy[:,0],xy[:,1],marker='x',s=16,color=orange,linewidths=.8,zorder=5)
        ax.set(xlim=(0,5120),ylim=(0,5120),xlabel='东向 / m',ylabel='北向 / m',aspect='equal')
        ax.set_xticks(np.arange(0,5001,1000));ax.set_yticks(np.arange(0,5001,1000))
        ax.tick_params(labelsize=9);ax.set_title(f'{title}：{totals[label]} / {required} 处连接通过',fontsize=12,pad=15,color=ink,loc='left')
        ax.annotate(letter,xy=(0,1),xycoords='axes fraction',xytext=(-18,16),textcoords='offset points',fontweight='bold',fontsize=12)
    fig.suptitle('A5 · 导航域分离：增加掉头水域，保留完整覆盖目标',fontsize=18,color=ink,y=.98)
    fig.text(.5,.929,'覆盖目标固定 8.767 km²；安全导航水域扩大至 17.628 km²；实体障碍裕度保持 50 m',ha='center',fontsize=11,color='#586b7d')
    fig.legend(handles=[Patch(facecolor='#dce8f3',label='原覆盖目标'),Patch(facecolor='#cbe9e6',label='新增安全导航水域'),
                        Patch(facecolor='#8b929a',label='实体障碍'),Line2D([],[],color=green,label='通过的曲线连接'),
                        Line2D([],[],color=orange,marker='x',ls='none',label='失败连接的端点')],
               loc='upper center',bbox_to_anchor=(.5,.90),ncol=5,frameon=False,fontsize=9)
    for x,label in zip((.28,.76),('original_domain','separated_domain')):
        count=trial['connector_counts'][label]
        text='　 '.join(f'{name.upper().replace("_","")}：{count[name]["passed"]}/{count[name]["required"]}'for name in ('uuv_1','uuv_2','usv'))
        fig.text(x,.16,text,ha='center',fontsize=10,color=ink)
    fig.text(.5,.112,f'余下 {required-totals["separated_domain"]} 处直接连接失败；其中 {audit["independently_verified_obstruction_count"]} 处原端点可证明缺少前进转弯空间。',ha='center',fontsize=11,color=ink)
    fig.text(.5,.071,f'图示为全部 {required} 处固定端点连接的逐项核查，未拼接为完整任务航迹；失败段未被删除。',ha='center',fontsize=10,color='#586b7d')
    fig.text(.5,.032,'浅灰底表示未纳入当前安全导航的区域，不能一概视为陆地；端点调整试验也尚未形成完整 UUV 路线。',ha='center',fontsize=9,color='#586b7d')
    fig.canvas.draw();sys.path.insert(0,str(args.qa_script_dir))
    from audit_panel_alignment import require_matplotlib_panel_alignment
    require_matplotlib_panel_alignment(fig,json_out=args.output_dir/'alignment.json',require_panel_labels=True,strict=True)
    stem=args.output_dir/'headland_navigation_a5'
    fig.savefig(stem.with_suffix('.png'),dpi=300)
    fig.savefig(stem.with_suffix('.pdf'))
    fig.savefig(stem.with_suffix('.svg'))
    plt.close(fig)
    manifest={'source_result_sha256':result_hash,'audit_sha256':hashlib.sha256(args.audit_json.read_bytes()).hexdigest(),
              'plot_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'connector_count_per_domain':len(trial['connections']),
              'geometry_downsampling':False,'display_bounds_m':[0,0,5120,5120],
              'output_sha256':{ext:hashlib.sha256(stem.with_suffix('.'+ext).read_bytes()).hexdigest()for ext in ('png','pdf','svg')}}
    (args.output_dir/'figure_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(stem.with_suffix('.png'))


if __name__=='__main__':main()
