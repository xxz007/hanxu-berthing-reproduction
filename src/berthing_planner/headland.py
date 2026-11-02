"""Finite scan variants and conservative forward reachable-position boxes."""
import math


def reachable_boxes(pose,radius,length_limit):
    if len(pose)!=3 or not all(math.isfinite(v)for v in (*pose,radius,length_limit))or radius<=0 or length_limit<0:
        raise ValueError('Finite pose, positive radius and nonnegative length limit required')
    yaw=pose[2]
    for s in (.5,1,2,3,5,8,10,15,20,25,30):
        if s>=min(length_limit,math.pi*radius/2)-1e-7:continue
        lo=radius*math.sin(s/radius);dy=radius*(1-math.cos(s/radius))
        ring=[[pose[0]+x*math.cos(yaw)-y*math.sin(yaw),pose[1]+x*math.sin(yaw)+y*math.cos(yaw)]
              for x,y in [(lo,-dy),(s,-dy),(s,dy),(lo,dy),(lo,-dy)]]
        yield s,{'type':'Polygon','coordinates':[ring]}


def scan_variants(points,radius,tangent_heading=None,blocked_entry=False,blocked_exit=False):
    if len(points)!=2 or any(len(p)!=2 for p in points)or not all(math.isfinite(v)for p in points for v in p)or not math.isfinite(radius)or radius<=0:
        raise ValueError('Finite scan endpoints and positive radius required')
    a,b=points;length=math.dist(a,b)
    if length<=0:raise ValueError('A positive-length scan is required')
    unit=[(b[j]-a[j])/length for j in (0,1)];heading=math.atan2(unit[1],unit[0])
    variants=[];seen=set()
    def add(p,q,kind):
        signature=tuple(round(v,8)for v in (*p,*q))
        if signature in seen:return
        seen.add(signature)
        variants.append({'kind':kind,'points':[list(p),list(q)],'length_m':math.dist(p,q),
                         'heading_rad':math.atan2(q[1]-p[1],q[0]-p[0])})
    add(a,b,'original');add(b,a,'reversed')
    if length>=4*radius:
        mask=(int(blocked_entry),int(blocked_exit))if blocked_entry or blocked_exit else(1,1)
        for amount in (10,20,30,40):
            trim=min(amount,.4*length)
            add([a[j]+mask[0]*trim*unit[j]for j in (0,1)],
                [b[j]-mask[1]*trim*unit[j]for j in (0,1)],'retreated')
        for amount in (20,40):
            add([a[j]-amount*unit[j]for j in (0,1)],[b[j]+amount*unit[j]for j in (0,1)],'extended')
        for amount in (20,40):
            trim=min(amount,.4*length)
            add([a[j]+trim*unit[j]for j in (0,1)],[b[j]-trim*unit[j]for j in (0,1)],'retreated_both')
    else:
        centre=[(a[j]+b[j])/2 for j in (0,1)]
        headings=[heading,heading+math.pi,heading+math.pi/2,heading-math.pi/2]
        if tangent_heading is not None:
            headings.extend(tangent_heading+turn+offset for turn in (0,math.pi)for offset in (0,math.pi/8,-math.pi/8))
        for yaw in headings:
            for scan_length in (max(length,2*radius),max(length,4*radius)):
                d=[math.cos(yaw),math.sin(yaw)]
                add([centre[j]-scan_length*d[j]/2 for j in (0,1)],
                    [centre[j]+scan_length*d[j]/2 for j in (0,1)],'short_through_pass')
    return variants


def select_variant_chain(layers,edge_function,fallback=None):
    """Shortest finite layered graph; retain all tasks, including failed layers.

    A node supplies nonnegative node_cost; an edge supplies length_m. Optional
    fallback edges are tried only if a complete next layer is unreachable.
    This is optimal only in the tested finite graph, not in continuous space.
    """
    if not layers or any(not layer for layer in layers):
        return {'passed':False,'empty_layers':[i for i,layer in enumerate(layers)if not layer]}
    scores=[v['node_cost']for v in layers[0]];parents=[]
    for index in range(1,len(layers)):
        current=[math.inf]*len(layers[index]);back=[None]*len(current)
        for function in [edge_function]+([fallback]if fallback else[]):
            for j,node in enumerate(layers[index]):
                for i,prior in enumerate(layers[index-1]):
                    if not math.isfinite(scores[i]):continue
                    edge=function(index,i,j,prior,node)
                    if edge is None:continue
                    value=scores[i]+node['node_cost']+edge['length_m']
                    if value<current[j]:current[j]=value;back[j]=(i,edge)
            if any(math.isfinite(v)for v in current):break
        if not any(math.isfinite(v)for v in current):
            return {'passed':False,'failed_layer':index,'layers_reached':index}
        scores=current;parents.append(back)
    selected=min(range(len(scores)),key=lambda i:scores[i]);total=scores[selected];indices=[selected];edges=[]
    for back in reversed(parents):
        selected,edge=back[selected];indices.append(selected);edges.append(edge)
    return {'passed':True,'variant_indices':indices[::-1],'edges':edges[::-1],'objective_cost':total}
