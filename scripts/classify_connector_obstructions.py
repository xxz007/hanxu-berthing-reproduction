"""Certify locally blocked forward departure by an outer reachable rectangle.

For heading0, curvature<=1/R and arc length s<=pi*R/2 imply
x in[R*sin(s/R),s], |y|<=R*(1-cos(s/R)). If this entire rectangle is
strictly separated from navigation, no admissible path can reach that s.
Only s below a proven start-goal path-length lower bound is considered.
"""
import argparse
import json
import math
from pathlib import Path

from chart_geometry import Geos


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial-dir',type=Path,required=True)
    parser.add_argument('--navigation-dir',type=Path,required=True)
    parser.add_argument('--json-out',type=Path,required=True)
    args=parser.parse_args();trial=json.loads((args.trial_dir/'result.json').read_text())
    nav=json.loads((args.navigation_dir/'navigation.geojson').read_text())['features'][0]['geometry']
    classified=[]
    with Geos()as g:
        for r in trial['connections']:
            if r['separated_domain']['passed']:continue
            start,goal=r['start_pose'],r['goal_pose'];radius=r['minimum_radius_m']
            lower=max(math.dist(start[:2],goal[:2]),radius*abs(math.remainder(goal[2]-start[2],2*math.pi)))
            certificates=[]
            for side,pose in [('exit',start),('entry',[*goal[:2],goal[2]+math.pi])]:
                for s in (.5,1,2,3,5,8,10,15,20,25,30):
                    if s>=min(lower,math.pi*radius/2)-1e-7:continue
                    lo=radius*math.sin(s/radius);dy=radius*(1-math.cos(s/radius));yaw=pose[2]
                    xy=[[pose[0]+x*math.cos(yaw)-y*math.sin(yaw),pose[1]+x*math.sin(yaw)+y*math.cos(yaw)]for x,y in [(lo,-dy),(s,-dy),(s,dy),(lo,dy),(lo,-dy)]]
                    rectangle={'type':'Polygon','coordinates':[xy]};distance=g.distance(rectangle,nav)
                    if distance>1e-5:
                        certificates.append({'side':side,'arc_length_m':s,'path_length_lower_bound_m':lower,
                                             'reachable_box':rectangle,'strict_distance_from_navigation_m':distance})
                        break
            classified.append({'platform':r['platform'],'source_leg_index':r['source_leg_index'],
                               'prior_task_id':r.get('prior_task_id'),'next_task_id':r.get('next_task_id'),
                               'certified_endpoint_obstructions':certificates})
    report={'failed_direct_connectors':len(classified),'connectors_with_proven_endpoint_obstruction':sum(bool(r['certified_endpoint_obstructions'])for r in classified),
            'records':classified,'scope':'Certificate applies to unchanged source poses and fixed navigation. Other failures remain candidate-search failures, not proof of infeasibility.'}
    args.json_out.write_text(json.dumps(report,indent=2)+'\n');print({k:v for k,v in report.items()if k!='records'})


if __name__=='__main__':main()
