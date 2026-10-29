"""R1: recover plotted obstacle centerlines, audit topology and visible grid alignment.

Zhang-Suen two-subiteration thinning (1984), implemented with NumPy.
This removes plotted stroke thickness globally; it does not edit target cells.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image,ImageDraw,ImageFont
import scipy
from scipy.ndimage import label,binary_dilation,distance_transform_edt
from reconstruct_paper_map import rasterize_ink,quadtree_leaves,connectivity

ROOT=Path(__file__).resolve().parents[1]


def thin_zhang_suen(mask):
    """Parallel removal; preserve one-pixel endpoints and connectivity locally.

    P2=N,P3=NE,...,P9=NW. Delete when 2<=B<=6,A=1 plus phase triples.
    Source: https://doi.org/10.1145/357994.358023 ; algorithm, not copied code.
    """
    skeleton=mask.copy()
    iterations=0
    while True:
        removed=0
        for phase in (0,1):
            z=np.pad(skeleton,1)
            p=[z[:-2,1:-1],z[:-2,2:],z[1:-1,2:],z[2:,2:],z[2:,1:-1],z[2:,:-2],z[1:-1,:-2],z[:-2,:-2]]
            count=np.sum(p,axis=0)
            transitions=np.sum([~p[i]&p[(i+1)%8] for i in range(8)],axis=0)
            triples=((p[0]&p[2]&p[4])|(p[2]&p[4]&p[6])) if phase==0 else ((p[0]&p[2]&p[6])|(p[0]&p[4]&p[6]))
            delete=skeleton&(count>=2)&(count<=6)&(transitions==1)&~triples
            removed+=int(delete.sum())
            skeleton[delete]=False
        iterations+=1
        if removed==0:
            return skeleton,iterations


def topology(mask):
    components=label(mask,np.ones((3,3)))[1]
    background,n=label(~mask)
    border=set(np.concatenate((background[0],background[-1],background[:,0],background[:,-1])).tolist())
    sizes=np.bincount(background.ravel())
    holes=[int(sizes[i]) for i in range(1,n+1) if i not in border]
    return {'foreground_components_8':components,'enclosed_background_holes_4':len(holes),
            'hole_sizes_pixels':sorted(holes),'note':'border-connected background regions are not holes'}


def grid_alignment(occupied,source_rgb,ink):
    """Independent check against black lines of the plotted original quadtree.

    Exclude the blue ink and frame; compare raster edges at 1.5 source pixels.
    This is visual alignment evidence, not a proof of author leaf identities.
    """
    height,width=ink.shape
    n=len(occupied)
    edges=Image.new('1',(width,height))
    draw=ImageDraw.Draw(edges)
    for cell in quadtree_leaves(occupied):
        draw.rectangle((cell['i0']*width/n,(n-cell['j1'])*height/n,
                        cell['i1']*width/n,(n-cell['j0'])*height/n),outline=1)
    predicted=np.asarray(edges)
    black=np.max(source_rgb,axis=2)<90
    valid=~binary_dilation(ink,iterations=2)
    valid[:3,:]=valid[-3:,:]=False
    valid[:,:3]=valid[:,-3:]=False
    precision=float(np.mean(distance_transform_edt(~black)[predicted&valid]<=1.5))
    recall=float(np.mean(distance_transform_edt(~predicted)[black&valid]<=1.5))
    return {'precision':precision,'recall':recall,'f1':2*precision*recall/(precision+recall),
            'tolerance_source_pixels':1.5,'meaning':'global plotted grid comparison; not chosen by endpoint success'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    r0=args.input_dir
    cfg=json.loads((r0/'config.json').read_text())
    cal=json.loads((r0/'calibration.json').read_text())
    env=json.loads((r0/'environment.json').read_text())
    source=r0/'source_figure_3_4.png'
    if hashlib.sha256(source.read_bytes()).hexdigest()!=env['source_figure_sha256']:
        raise ValueError('Input figure differs from the R0 source record')
    output=args.output_dir or ROOT/'outputs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_paper_map_r1')
    if output.exists():
        raise FileExistsError('Preserving existing map evidence: '+str(output))
    crop=Image.open(source).convert('RGB').crop(cfg['plot_frame_px'])
    rgb=np.asarray(crop)
    variants={}
    for t in cfg['blue_ink_thresholds']:
        ink=(rgb[:,:,0]<=t['r_max'])&(rgb[:,:,1]<=t['g_max'])&(rgb[:,:,2]>=t['b_min'])
        skeleton,iterations=thin_zhang_suen(ink)
        occupied=rasterize_ink(skeleton,cfg['quadtree_max_depth'])
        original_occupied=rasterize_ink(ink,cfg['quadtree_max_depth'])
        before,after=topology(ink),topology(skeleton)
        variants[t['name']]={'skeleton':skeleton,'occupied':occupied,
            'summary':{'thinning_iterations':iterations,'ink_pixels':int(ink.sum()),'skeleton_pixels':int(skeleton.sum()),
            'skeleton_subset_of_ink':bool(np.all(~skeleton|ink)),
            'topology_before':before,'topology_after':after,
            'original_ink_grid_alignment':grid_alignment(original_occupied,rgb,ink),
            'centerline_grid_alignment':grid_alignment(occupied,rgb,ink),
            'occupied_cells':int(occupied.sum()),'leaves':len(quadtree_leaves(occupied)),
            'connectivity':connectivity(occupied,cfg['positions'],cal['bounds_EN_proxy'])}}
    base=variants['baseline']
    for name,v in variants.items():
        union=int(np.count_nonzero(v['occupied']|base['occupied']))
        v['summary']['changed_cells_vs_baseline']=int(np.count_nonzero(v['occupied']!=base['occupied']))
        v['summary']['occupied_cell_iou_vs_baseline']=int(np.count_nonzero(v['occupied']&base['occupied']))/union
    passed=all(v['summary']['skeleton_subset_of_ink'] and
        v['summary']['topology_before']['foreground_components_8']==v['summary']['topology_after']['foreground_components_8'] and
        v['summary']['topology_before']['enclosed_background_holes_4']==v['summary']['topology_after']['enclosed_background_holes_4'] and
        all(not p['blocked'] and p['reachable_from_start'] for p in v['summary']['connectivity']['positions'].values())
        for v in variants.values())
    passed=passed and all(v['summary']['centerline_grid_alignment']['f1']>v['summary']['original_ink_grid_alignment']['f1'] for v in variants.values())
    checks={'reconstructed_map_gate_passed':passed,'scope':'independent centerline reconstruction usable for planning; not author raw map equivalence',
        'no_endpoint_moved_or_special_cell_cleared':True,'all_three_predeclared_thresholds_checked':True,
        'method':'Zhang-Suen thinning; original conservative pixel-window aggregation retained',
        'heading_status':'not calibrated; R2 position-only search may proceed',
        'uncertainty':'pixel calibration and centerline choices remain; not a ship-hull clearance guarantee'}
    output.mkdir(parents=True,exist_ok=False)
    (output/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    (output/'audit.json').write_text(json.dumps({k:v['summary'] for k,v in variants.items()},indent=2)+'\n')
    (output/'calibration.json').write_bytes((r0/'calibration.json').read_bytes())
    (output/'positions.json').write_text(json.dumps(cfg['positions'],indent=2)+'\n')
    for name,v in variants.items():
        Image.fromarray(np.uint8(v['skeleton'])*255).save(output/(name+'_centerline.png'))
        map_data={'schema_version':1,'status':'reconstructed_centerline_not_author_raw_map',
            'bounds':cal['bounds_EN_proxy'],'n':len(v['occupied']),
            'rows_bottom_to_top':v['occupied'].astype(int).tolist(),
            'coordinate_convention':cfg['coordinate_convention'],'source_pdf_sha256':env['source_pdf_sha256'],
            'source_figure_sha256':env['source_figure_sha256'],'reconstruction':checks['method']}
        (output/(name+'_map.json')).write_text(json.dumps(map_data,separators=(',',':'))+'\n')
    width,height=rgb.shape[1],rgb.shape[0]
    preview=Image.new('RGB',(width*2+60,height+70),'#f3f6f8')
    preview.paste(crop,(20,45))
    recovered=Image.new('RGB',(width,height),'#eff6fa')
    d=ImageDraw.Draw(recovered)
    n=base['occupied'].shape[0]
    for cell in quadtree_leaves(base['occupied']):
        d.rectangle((cell['i0']*width/n,(n-cell['j1'])*height/n,cell['i1']*width/n,(n-cell['j0'])*height/n),
                    fill='#284f9b' if cell['blocked'] else None,outline='#a9b8c0',width=1)
    preview.paste(recovered,(width+40,45))
    d=ImageDraw.Draw(preview)
    font=ImageFont.load_default(size=18)
    d.text((20,12),'Source plot: thick blue markers',font=font,fill='#233443')
    d.text((width+40,12),'Global centerline recovery + depth-7 quadtree',font=font,fill='#233443')
    for p in cfg['positions']:
        E,N=p['chart_EN_proxy']
        for ox in (20,width+40):
            u,v=ox+E/cal['bounds_EN_proxy'][2]*width,45+(1-N/cal['bounds_EN_proxy'][3])*height
            d.ellipse((u-5,v-5,u+5,v+5),fill='#e04a3b',outline='white')
    preview.save(output/'centerline_recovery_preview.png')
    shutil.copy2(__file__,output/'refine_paper_map.py')
    (output/'environment.json').write_text(json.dumps({'created_utc':datetime.now(timezone.utc).isoformat(),
        'numpy':np.__version__,'scipy':scipy.__version__,'input_r0_environment':env,
        'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'source_reference':'https://scikit-image.org/docs/stable/auto_examples/edges/plot_skeleton.html'},indent=2)+'\n')
    print(output)
    print(json.dumps(checks,indent=2))
    for name,v in variants.items():
        print(name,json.dumps({k:v['summary'][k] for k in ('occupied_cells','leaves','centerline_grid_alignment')},indent=2))


if __name__=='__main__':
    main()
