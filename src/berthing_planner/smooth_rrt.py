"""Yu et al. ICMA 2017: goal bias, bounded turn, greedy polyline smoothing.

The primary source does not provide continuous-curvature interpolation. Eq.(5)
is ambiguous; directional angle clipping with a step-length cap is an explicit
reconstruction. Root yaw and the future-join greedy guard are also explicit.
"""
import math
import random
from .curves import wrap


def steer(point,yaw,target,step,maximum_turn):
    desired=math.atan2(target[1]-point[1],target[0]-point[0])
    yaw=wrap(yaw+max(-maximum_turn,min(maximum_turn,wrap(desired-yaw))))
    length=min(step,math.dist(point,target))
    return (point[0]+length*math.cos(yaw),point[1]+length*math.sin(yaw)),yaw


def greedy_path(tree,path,initial_yaw,maximum_turn,policy='preserve_both_joins'):
    """Advance until the first failed shortcut, as described on pp.1499-1500.

    Incoming-only mirrors the stated local predicate but can strand the next
    step after a deletion. The both-joins interpretation checks the immediate
    untouched outgoing edge as well, preserving the global turn constraint.
    """
    if policy not in ('preserve_both_joins','incoming_only'):
        raise ValueError('Unknown greedy interpretation')
    result=[path[0]];indices=[0];i=0;yaw=initial_yaw;trace=[]
    while i<len(path)-1:
        chosen=None
        for j in range(i+1,len(path)):
            heading=math.atan2(path[j][1]-path[i][1],path[j][0]-path[i][0])
            valid=abs(wrap(heading-yaw))<=maximum_turn+1e-10 and tree.line_of_sight(path[i],path[j])
            if valid and policy=='preserve_both_joins' and j<len(path)-1:
                outgoing=math.atan2(path[j+1][1]-path[j][1],path[j+1][0]-path[j][0])
                valid=abs(wrap(outgoing-heading))<=maximum_turn+1e-10
            if not valid:break
            chosen=j
        if chosen is None:
            return {'status':'stalled_after_shortcut','partial_path':result,'path':None,
                    'retained_indices':indices,'trace':trace,'policy':policy}
        trace.append({'from_index':i,'to_index':chosen})
        yaw=math.atan2(path[chosen][1]-path[i][1],path[chosen][0]-path[i][0])
        result.append(path[chosen]);indices.append(chosen);i=chosen
    return {'status':'smoothed_polyline','path':result,'retained_indices':indices,
            'trace':trace,'policy':policy}


def smooth_rrt(tree,start,goal,*,step=200.0,maximum_turn=math.pi/6,goal_bias=.1,
               max_iterations=20000,seed=0,initial_yaw=0.0,greedy_policy='preserve_both_joins'):
    if not (math.isfinite(step) and step>0 and 0<maximum_turn<=math.pi and 0<goal_bias<1
            and type(max_iterations) is int and max_iterations>0):
        raise ValueError('Invalid RRT step, angle, bias or iteration budget')
    start,goal=tuple(start),tuple(goal)
    if not tree.line_of_sight(start,start) or not tree.line_of_sight(goal,goal):
        raise ValueError('Endpoint touches a blocked cell')
    nodes=[start];headings=[initial_yaw];parents=[0];poses={(start[0],start[1],initial_yaw)}
    rng=random.Random(seed);x0,y0,x1,y1=tree.bounds
    rejected_collision=0;rejected_duplicate=0;found=None
    if start==goal:found=0
    for iteration in range(1,max_iterations+1):
        if found is not None:break
        target=goal if rng.random()<goal_bias else (rng.uniform(x0,x1),rng.uniform(y0,y1))
        near=min(range(len(nodes)),key=lambda i:(nodes[i][0]-target[0])**2+(nodes[i][1]-target[1])**2)
        point,yaw=steer(nodes[near],headings[near],target,step,maximum_turn)
        if math.dist(point,nodes[near])<1e-8 or (*point,yaw) in poses:
            rejected_duplicate+=1;continue
        if not(x0<=point[0]<x1 and y0<=point[1]<y1) or not tree.line_of_sight(nodes[near],point):
            rejected_collision+=1;continue
        nodes.append(point);headings.append(yaw);parents.append(near);poses.add((*point,yaw))
        index=len(nodes)-1
        if math.dist(point,goal)<1e-8:
            found=index;break
        if math.dist(point,goal)<=step:
            terminal_yaw=math.atan2(goal[1]-point[1],goal[0]-point[0])
            if abs(wrap(terminal_yaw-yaw))<=maximum_turn+1e-10 and tree.line_of_sight(point,goal):
                nodes.append(goal);headings.append(terminal_yaw);parents.append(index)
                found=len(nodes)-1;break
    stats={'seed':seed,'iterations':iteration,'tree_nodes':len(nodes),
           'rejected_collision':rejected_collision,'rejected_duplicate':rejected_duplicate,
           'step_m':step,'maximum_turn_rad':maximum_turn,'goal_bias':goal_bias,
           'initial_yaw_rad':initial_yaw,'greedy_policy':greedy_policy}
    if found is None:
        return {'status':'iteration_budget_exhausted','path':[],'raw_path':[],**stats}
    chain=[found]
    while chain[-1]!=0:chain.append(parents[chain[-1]])
    raw=[nodes[i] for i in reversed(chain)]
    smoothing=greedy_path(tree,raw,initial_yaw,maximum_turn,greedy_policy)
    path=smoothing['path'] or raw
    yaw=initial_yaw;turns=[]
    for a,b in zip(path,path[1:]):
        angle=math.atan2(b[1]-a[1],b[0]-a[0]);turns.append(abs(wrap(angle-yaw)));yaw=angle
    return {'status':'path_found' if smoothing['path'] else 'path_found_greedy_failed',
            'path':path,'raw_path':raw,'smoothing':smoothing,
            'length_m':sum(math.dist(a,b) for a,b in zip(path,path[1:])),
            'maximum_actual_turn_rad':max(turns,default=0),**stats}
