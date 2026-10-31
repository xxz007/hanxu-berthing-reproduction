"""Download and verify the two pinned NOAA chart archives; preserve changed inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/open_chart_long_beach_stage_a.json')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--noaa-cname', action='store_true',
                        help='Route this curl request through NOAA public AWS CNAME while retaining the official hostname and TLS verification')
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    args.output_dir.mkdir(parents=True, exist_ok=False)
    records = []
    for chart in cfg['charts']:
        target = args.output_dir/(chart['cell']+'.zip')
        if args.noaa_cname:
            command = ['curl','--fail','--location','--silent','--show-error','--max-time','45',
                       '--connect-to','www.charts.noaa.gov:443:www.charts.ocs-aws-prod.net:443',
                       '--output',str(target),chart['url']]
            result = subprocess.run(command,capture_output=True,text=True)
            if result.returncode:
                raise RuntimeError(result.stderr[:700])
        else:
            with urllib.request.urlopen(chart['url'],timeout=45) as response:
                target.write_bytes(response.read())
            command = ['urllib.request.urlopen',chart['url']]
        sha = hashlib.sha256(target.read_bytes()).hexdigest()
        if target.stat().st_size != chart['bytes'] or sha != chart['sha256']:
            raise ValueError('NOAA archive changed from the pinned snapshot; downloaded file preserved. Review a new source version before changing config.')
        records.append({'cell':chart['cell'],'url':chart['url'],'sha256':sha,'command':command})
        print('Verified',chart['cell'],target.stat().st_size,'bytes')
    (args.output_dir/'download_manifest.json').write_text(json.dumps(records,indent=2)+'\n')


if __name__ == '__main__':
    main()
