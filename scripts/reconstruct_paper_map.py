"""R0: extract thesis Fig.3.4, calibrate axes and reconstruct blue-ink occupancy.

Research preprocessing only: existing workbench PyMuPDF/NumPy/Pillow required.
No author data claim, no heading inference, no edits to PDF or v0.1 baseline.
All derived source imagery and map data stay in ignored outputs directories.
"""
import argparse
from collections import deque
import csv
from datetime import datetime,timezone
import hashlib
from io import BytesIO
import json
import math
from pathlib import Path
import platform
import shutil
import sys

import numpy as np
from PIL import Image,ImageDraw,ImageFont
import PIL
import pymupdf

ROOT=Path(__file__).resolve().parents[1]


def calibrate_ticks(ticks,origin,sign):
    pixels=np.array([p for p,_ in ticks],dtype=float)
    values=np.array([v for _,v in ticks],dtype=float)
    offsets=sign*(pixels-origin)
    scale=float(offsets@values/(offsets@offsets))
    residuals=offsets*scale-values
    return {'meters_per_pixel':scale,'ticks':ticks,'residuals_m':residuals.tolist(),
            'max_abs_residual_m':float(abs(residuals).max()),
            'rms_residual_m':float(np.sqrt(np.mean(residuals**2)))}


def rasterize_ink(mask,depth):
    """Conservative pixel-window aggregation; no erosion/closing or land filling."""
    n=2**depth
    height,width=mask.shape
    occupied=np.zeros((n,n),dtype=bool)  # row 0 at the bottom of the chart
    for j in range(n):
        top=math.floor((n-1-j)*height/n)
        bottom=math.ceil((n-j)*height/n)
        for i in range(n):
            left=math.floor(i*width/n)
            right=math.ceil((i+1)*width/n)
            occupied[j,i]=bool(mask[top:bottom,left:right].any())
    return occupied


def quadtree_leaves(occupied):
    n=occupied.shape[0]
    result=[]
    def split(i0,j0,i1,j1,depth,code):
        blocked=bool(occupied[j0:j1,i0:i1].any())
        if not blocked or i1-i0==1:
            result.append({'code':code,'depth':depth,'i0':i0,'j0':j0,'i1':i1,'j1':j1,'blocked':blocked})
            return
        mx,my=(i0+i1)//2,(j0+j1)//2
        for suffix,box in [('00',(i0,j0,mx,my)),('10',(mx,j0,i1,my)),('01',(i0,my,mx,j1)),('11',(mx,my,i1,j1))]:
            split(*box,depth+1,code+suffix)
    split(0,0,n,n,0,'r')
    return result


def connectivity(occupied,positions,bounds):
    n=occupied.shape[0]
    width,height=bounds[2],bounds[3]
    lookup={}
    for p in positions:
        E,N=p['chart_EN_proxy']
        i,j=math.floor(E/width*n),math.floor(N/height*n)
        if not 0<=i<n or not 0<=j<n:
            raise ValueError('Reference point outside recovered frame: '+p['name'])
        lookup[p['name']]={'cell':[i,j],'blocked':bool(occupied[j,i]),
                           'chart_EN_proxy':[E,N],'thesis_xy':p['thesis_xy']}
    start=tuple(lookup['start']['cell'])
    parent={start:None} if not lookup['start']['blocked'] else {}
    queue=deque(parent)
    while queue:
        i,j=queue.popleft()
        for a,b in ((i-1,j),(i+1,j),(i,j-1),(i,j+1)):
            if 0<=a<n and 0<=b<n and not occupied[b,a] and (a,b) not in parent:
                parent[a,b]=(i,j)
                queue.append((a,b))
    paths={}
    for name,p in lookup.items():
        target=tuple(p['cell'])
        p['reachable_from_start']=target in parent
        if name!='start' and target in parent:
            chain=[target]
            while parent[chain[-1]] is not None:
                chain.append(parent[chain[-1]])
            paths[name]=[[width*(i+.5)/n,height*(j+.5)/n] for i,j in reversed(chain)]
    return {'positions':lookup,'reachable_cells':len(parent),'diagnostic_bfs_paths_EN':paths,
            'meaning':'4-neighbor raster connectivity only; not A-star, not curvature/heading/ship feasibility'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-pdf',type=Path)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/paper_map_r0.json')
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    config=json.loads(args.config.read_text())
    if config['schema_version']!=1 or config['occupancy_rule']!='any_blue_ink_pixel_overlapping_cell_pixel_window':
        raise ValueError('Unsupported R0 schema/occupancy semantics')
    if not 1<=config['quadtree_max_depth']<=9:
        raise ValueError('R0 depth must be between 1 and 9')
    local=ROOT/'docs/source_local.json'
    if args.source_pdf is None and not local.exists():
        raise ValueError('Supply --source-pdf with the user-held thesis PDF')
    source=args.source_pdf or Path(json.loads(local.read_text())['thesis_pdf'])
    manifest=json.loads((ROOT/config['source']['manifest']).read_text())
    pdf_hash=hashlib.sha256(source.read_bytes()).hexdigest()
    if pdf_hash!=manifest['sha256']:
        raise ValueError('PDF hash differs from the verified thesis; re-identify figure before proceeding')
    output=args.output_dir or ROOT/'outputs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_paper_map_r0')
    if output.exists():
        raise FileExistsError('Preserving previous results: '+str(output))
    doc=pymupdf.open(source)
    info=config['source']
    if info['image_xref'] not in [x[0] for x in doc[info['pdf_physical_page']-1].get_images()]:
        raise ValueError('Image xref not on declared PDF page')
    extracted=doc.extract_image(info['image_xref'])
    image=Image.open(BytesIO(extracted['image'])).convert('RGB')
    if list(image.size)!=info['image_size_px']:
        raise ValueError('Figure size differs from reviewed source')
    frame=config['plot_frame_px']
    x0,y0,x1,y1=frame
    if not (0<=x0<x1<=image.width and 0<=y0<y1<=image.height):
        raise ValueError('Invalid figure plot frame')
    if config['calibration']['pixel_origin']!=[x0,y1]:
        raise ValueError('This R0 uses the frame lower-left as the zero tick')
    Efit=calibrate_ticks(config['calibration']['horizontal'],x0,1)
    Nfit=calibrate_ticks(config['calibration']['vertical'],y1,-1)
    Emax=(x1-x0)*Efit['meters_per_pixel']
    Nmax=(y1-y0)*Nfit['meters_per_pixel']
    bounds=[0,0,Emax,Nmax]
    cropped=image.crop(frame)
    rgb=np.asarray(cropped)
    n=2**config['quadtree_max_depth']
    variants={}
    for threshold in config['blue_ink_thresholds']:
        mask=(rgb[:,:,0]<=threshold['r_max'])&(rgb[:,:,1]<=threshold['g_max'])&(rgb[:,:,2]>=threshold['b_min'])
        occupied=rasterize_ink(mask,config['quadtree_max_depth'])
        variants[threshold['name']]={'mask':mask,'occupied':occupied,'connectivity':connectivity(occupied,config['positions'],bounds)}
    base=variants['baseline']
    leaves=quadtree_leaves(base['occupied'])
    covered=np.zeros((n,n),dtype=np.uint8)
    for leaf in leaves:
        covered[leaf['j0']:leaf['j1'],leaf['i0']:leaf['i1']]+=1
        if leaf['blocked'] != bool(base['occupied'][leaf['j0']:leaf['j1'],leaf['i0']:leaf['i1']].any()):
            raise AssertionError('Leaf occupancy does not match reconstructed raster')
    if not np.all(covered==1):
        raise AssertionError('Quadtree must tile the raster exactly once')
    calibration={'horizontal':Efit,'vertical':Nfit,'bounds_EN_proxy':bounds,
                 'minimum_cell_size_m':[Emax/n,Nmax/n],
                 'one_pixel_m':[Efit['meters_per_pixel'],Nfit['meters_per_pixel']],
                 'limitation':'tick fitting residuals exclude blue stroke width, chart accuracy and segmentation errors; heading/geography not calibrated'}
    ink_rows,ink_cols=np.where(base['mask'])
    point_diagnostics={}
    for p in config['positions']:
        E,N=p['chart_EN_proxy']
        u=x0+E/Efit['meters_per_pixel']
        v=y1-N/Nfit['meters_per_pixel']
        nearest=np.hypot((ink_cols+x0-u)*Efit['meters_per_pixel'],
                         (ink_rows+y0-v)*Nfit['meters_per_pixel'])
        source_pixel=np.asarray(image)[round(v),round(u)]
        t=next(t for t in config['blue_ink_thresholds'] if t['name']=='baseline')
        point_diagnostics[p['name']]={'source_pixel_uv':[u,v],
            'source_pixel_is_blue':bool(source_pixel[0]<=t['r_max'] and source_pixel[1]<=t['g_max'] and source_pixel[2]>=t['b_min']),
            'distance_to_nearest_blue_ink_m':float(nearest.min()),
            'aggregated_cell_blocked':base['connectivity']['positions'][p['name']]['blocked'],
            'meaning':'distance to plotted ink, not physical clearance to original shoreline'}
    sensitivity={}
    for name,v in variants.items():
        union=np.count_nonzero(v['occupied']|base['occupied'])
        intersection=np.count_nonzero(v['occupied']&base['occupied'])
        sensitivity[name]={'blue_pixels':int(v['mask'].sum()),'occupied_cells':int(v['occupied'].sum()),
                           'changed_cells_vs_baseline':int(np.count_nonzero(v['occupied']!=base['occupied'])),
                           'occupied_cell_iou_vs_baseline':intersection/union if union else 1.0,
                           'positions':v['connectivity']['positions']}
    output.mkdir(parents=True,exist_ok=False)
    (output/'source_figure_3_4.png').write_bytes(extracted['image'])
    Image.fromarray(np.uint8(base['mask'])*255).save(output/'blue_ink_mask.png')
    (output/'config.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
    (output/'calibration.json').write_text(json.dumps(calibration,ensure_ascii=False,indent=2)+'\n')
    (output/'sensitivity.json').write_text(json.dumps(sensitivity,ensure_ascii=False,indent=2)+'\n')
    (output/'connectivity.json').write_text(json.dumps(base['connectivity'],ensure_ascii=False,indent=2)+'\n')
    (output/'point_diagnostics.json').write_text(json.dumps(point_diagnostics,ensure_ascii=False,indent=2)+'\n')
    with (output/'occupied_cells_EN.csv').open('w',newline='') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['i','j','E_min_m','N_min_m','E_max_m','N_max_m'])
        for j,i in zip(*np.where(base['occupied'])):
            writer.writerow([i,j,i*Emax/n,j*Nmax/n,(i+1)*Emax/n,(j+1)*Nmax/n])
    with (output/'quadtree_leaves.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(leaves[0]),lineterminator='\n')
        writer.writeheader();writer.writerows(leaves)
    (output/'occupancy.json').write_text(json.dumps({'status':config['status'],'bounds':bounds,
           'n':n,'rows_bottom_to_top':base['occupied'].astype(int).tolist()},separators=(',',':'))+'\n')
    # Local evidence preview only; do not publish source-derived imagery automatically.
    W,H=980,410
    preview=Image.new('RGB',(W,H),'#f3f6f8')
    draw=ImageDraw.Draw(preview)
    font=ImageFont.load_default(size=15)
    panel_w,panel_h=450,348
    source_view=cropped.resize((panel_w,panel_h))
    preview.paste(source_view,(25,45))
    reconstructed=Image.new('RGB',cropped.size,'#f0f7fa')
    d=ImageDraw.Draw(reconstructed)
    sx,sy=cropped.width/n,cropped.height/n
    for leaf in leaves:
        box=(leaf['i0']*sx,(n-leaf['j1'])*sy,leaf['i1']*sx,(n-leaf['j0'])*sy)
        d.rectangle(box,fill='#284f9b' if leaf['blocked'] else None,outline='#abb8be',width=1)
    preview.paste(reconstructed.resize((panel_w,panel_h)),(505,45))
    for p in config['positions']:
        E,N=p['chart_EN_proxy']
        for ox in (25,505):
            u,v=ox+E/Emax*panel_w,45+(1-N/Nmax)*panel_h
            draw.ellipse((u-4,v-4,u+4,v+4),fill='#f44336',outline='white')
    draw.text((25,15),'Thesis Fig.3.4: source plot (local only)',font=font,fill='#233443')
    draw.text((505,15),'R0: blue-ink occupancy + depth-7 quadtree',font=font,fill='#233443')
    preview.save(output/'map_reconstruction_preview.png')
    annotated=image.copy()
    d=ImageDraw.Draw(annotated)
    d.rectangle(frame,outline='red',width=2)
    for u,value in config['calibration']['horizontal']:
        d.ellipse((u-3,y1-11,u+3,y1-5),fill='red')
    for v,value in config['calibration']['vertical']:
        d.ellipse((x0+3,v-3,x0+9,v+3),fill='red')
    annotated.save(output/'calibration_overlay.png')
    for p in config['positions']:
        u,v=point_diagnostics[p['name']]['source_pixel_uv']
        i,j=base['connectivity']['positions'][p['name']]['cell']
        detail=image.copy()
        d=ImageDraw.Draw(detail)
        d.rectangle((x0+i*(x1-x0)/n,y0+(n-1-j)*(y1-y0)/n,
                     x0+(i+1)*(x1-x0)/n,y0+(n-j)*(y1-y0)/n),outline='red',width=1)
        d.ellipse((u-1,v-1,u+1,v+1),fill='lime')
        detail.crop((int(u)-35,int(v)-35,int(u)+35,int(v)+35)).resize((420,420)).save(output/(p['name']+'_detail.png'))
    checks={'source_hash_verified':True,'exactly_one_leaf_per_fine_cell':bool(np.all(covered==1)),
            'leaf_count':len(leaves),'blocked_leaves':sum(v['blocked'] for v in leaves),
            'free_leaves':sum(not v['blocked'] for v in leaves),
            'fine_cells':n*n,'occupied_fine_cells':int(base['occupied'].sum()),
            'all_reference_points_free':all(not p['blocked'] for p in base['connectivity']['positions'].values()),
            'both_goals_4_neighbor_reachable':all(p['reachable_from_start'] for p in base['connectivity']['positions'].values()),
            'scope':'image reconstruction and raster connectivity only; not a thesis planning reproduction pass'}
    (output/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    shutil.copy2(__file__,output/'reconstruct_paper_map.py')
    environment={'created_utc':datetime.now(timezone.utc).isoformat(),'python':platform.python_version(),
                 'pymupdf':pymupdf.VersionBind,'numpy':np.__version__,'pillow':PIL.__version__,
                 'source_pdf_sha256':pdf_hash,'source_figure_sha256':hashlib.sha256(extracted['image']).hexdigest(),
                 'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                 'config_sha256':hashlib.sha256(args.config.read_bytes()).hexdigest(),
                 'runtime':'existing research workbench environment; project stdlib venv unchanged'}
    (output/'environment.json').write_text(json.dumps(environment,indent=2)+'\n')
    print(output)
    print(json.dumps({'calibration':calibration,'checks':checks,'sensitivity':sensitivity},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
