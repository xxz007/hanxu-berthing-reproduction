"""Fetch the pinned public GSHHG 2.3.7 binary release without installing GMT."""
import argparse
import hashlib
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
URL = 'https://github.com/GenericMappingTools/gshhg-gmt/releases/download/2.3.7/gshhg-bin-2.3.7.zip'
SHA256 = '28600e8f7a08645aab43079326df6504212ec5ccb2b4bcf3b5f4f12ed60e82bc'
SIZE = 118617033


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'data/raw/gshhg-bin-2.3.7.zip')
    args = parser.parse_args()
    path = args.output
    if path.exists():
        if path.stat().st_size != SIZE or hashlib.sha256(path.read_bytes()).hexdigest() != SHA256:
            raise ValueError('Existing archive does not match the pinned release; preserved unchanged')
        print('Verified existing GSHHG archive:', path)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_suffix('.zip.partial')
    digest = hashlib.sha256()
    request = urllib.request.Request(URL, headers={'User-Agent': 'HanXu-Chapter3-reproduction'})
    with partial.open('xb') as target, urllib.request.urlopen(request, timeout=60) as source:
        while chunk := source.read(1024*1024):
            target.write(chunk)
            digest.update(chunk)
    if partial.stat().st_size != SIZE or digest.hexdigest() != SHA256:
        raise ValueError('Archive checksum or size failed; partial file retained for diagnosis')
    partial.rename(path)
    print('Downloaded and SHA256-verified GSHHG archive:', path)


if __name__ == '__main__':
    main()
