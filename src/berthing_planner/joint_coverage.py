"""Finite A6 scan-position, heading and task-order candidates.

These are explicit engineering candidates, not an author algorithm or a
complete continuous-space optimizer. Every source task identifier is retained.
"""
import math
from .headland import scan_variants


def expanded_scan_variants(points,radius):
    variants=scan_variants(points,radius);a,b=points;length=math.dist(a,b)
    if length<4*radius:
        centre=[(a[j]+b[j])/2 for j in (0,1)]
        offsets=[(0,0)]+[(d*math.cos(i*math.pi/4),d*math.sin(i*math.pi/4))for d in (5,10,20,30)for i in range(8)]
        for dx,dy in offsets:
            for i in range(32):
                angle=i*math.pi/16;unit=(math.cos(angle),math.sin(angle))
                for size in sorted({length,radius,2*radius,4*radius}):
                    p=[centre[0]+dx-size*unit[0]/2,centre[1]+dy-size*unit[1]/2]
                    q=[centre[0]+dx+size*unit[0]/2,centre[1]+dy+size*unit[1]/2]
                    variants.append({'points':[p,q],'kind':'short_shift_rotate','length_m':size,'heading_rad':angle})
    else:
        unit=[(b[j]-a[j])/length for j in (0,1)]
        for first in (0,10,20,30,40,60,80):
            for last in (0,10,20,30,40,60,80):
                if first+last>=length*.8:continue
                p=[a[j]+first*unit[j]for j in (0,1)];q=[b[j]-last*unit[j]for j in (0,1)]
                variants.append({'points':[p,q],'kind':'independent_endpoint_retreat','length_m':math.dist(p,q),'heading_rad':math.atan2(unit[1],unit[0])})
    # Reversing every adapted scan is essential: reversing only the original
    # scan leaves many valid orientation/endpoint combinations unexplored.
    unique={}
    for v in variants:
        for reverse in (False,True):
            p,q=v['points'][::-1]if reverse else v['points']
            signature=tuple(round(x,8)for x in (*p,*q))
            if signature not in unique:
                unique[signature]={**v,'points':[p,q],'heading_rad':math.atan2(q[1]-p[1],q[0]-p[0]),
                                   'kind':v['kind']+('_reversed'if reverse else '')}
    return list(unique.values())


def task_order_candidates(tasks,radius=None):
    ids=[t['task_id']for t in tasks]
    if len(set(ids))!=len(ids):raise ValueError('Unique task identifiers required')
    rows=sorted(tasks,key=lambda t:((t['points'][0][1]+t['points'][1][1])/2,(t['points'][0][0]+t['points'][1][0])/2,t['task_id']))
    choices=[('source_order',ids)]
    if radius is not None:
        short={t['task_id']:t for t in tasks if math.dist(*t['points'])<4*radius}
        seen_clusters=set()
        for seed in sorted(short):
            if math.dist(*short[seed]['points'])>radius/2:continue
            cluster={seed};frontier=[seed]
            while frontier:
                current=short[frontier.pop()];centre=[sum(p[j]for p in current['points'])/2 for j in (0,1)]
                for key,t in short.items():
                    other=[sum(p[j]for p in t['points'])/2 for j in (0,1)]
                    if key not in cluster and math.dist(centre,other)<=8*radius:cluster.add(key);frontier.append(key)
            signature=tuple(sorted(cluster))
            if signature in seen_clusters:continue
            seen_clusters.add(signature);group=[t for t in ids if t in cluster];rest=[t for t in ids if t not in cluster]
            choices.extend([(f'short_cluster_last_{seed}',rest+group),(f'short_cluster_first_{seed}',group[::-1]+rest)])
    choices.append(('geographic_order',[t['task_id']for t in rows]))
    # Opposite progression changes which short passes are adjacent and which
    # ends may serve as distributed initial/final deployment points.
    choices.extend((name+'_reversed',list(reversed(order)))for name,order in choices[:])
    seen=set();result=[]
    for name,order in choices:
        if tuple(order)not in seen:result.append((name,order));seen.add(tuple(order))
    return result


def plan_variant_order(layers,edge_stages):
    """Layered graph with stage-wide escalation, not per-pair escalation.

    Exhaust a cheaper connection stage for the whole layer before invoking
    the next stage. Preserve all reachable alternatives within that stage.
    """
    if not layers or any(not layer for layer in layers):return {'passed':False,'empty_layers':[i for i,v in enumerate(layers)if not v]}
    scores=[v['node_cost']for v in layers[0]];parents=[]
    for k in range(1,len(layers)):
        current=[math.inf]*len(layers[k]);back=[None]*len(current)
        for function in edge_stages:
            for j,node in enumerate(layers[k]):
                for i,prior in enumerate(layers[k-1]):
                    if not math.isfinite(scores[i]):continue
                    edge=function(k,i,j,prior,node)
                    if edge is None:continue
                    value=scores[i]+node['node_cost']+edge['length_m']
                    if value<current[j]:current[j]=value;back[j]=(i,edge)
            if any(math.isfinite(v)for v in current):break
        if not any(math.isfinite(v)for v in current):return {'passed':False,'failed_layer':k,'layers_reached':k}
        scores=current;parents.append(back)
    selected=min(range(len(scores)),key=lambda i:scores[i]);total=scores[selected];indices=[selected];edges=[]
    for back in reversed(parents):selected,edge=back[selected];indices.append(selected);edges.append(edge)
    return {'passed':True,'variant_indices':indices[::-1],'edges':edges[::-1],'objective_cost':total}
