"""Check A10 output hashes, completed dependency order and frozen scientific results."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from run_reproduction import commands, digest, verify_archives


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--json-out', type=Path, required=True)
    args = parser.parse_args(); out = args.run_dir.resolve()
    read = lambda p: json.loads(p.read_text())
    manifest = read(out/'manifest.json'); result = read(out/'acceptance.json'); errors = []
    if manifest['status'] != 'passed' or not result['passed'] or digest(out/'acceptance.json') != manifest['acceptance_sha256']:
        errors.append('completed acceptance binding')
    for rel, expected in result['output_sha256'].items():
        if digest(out/rel) != expected:
            errors.append('output changed: ' + rel)
    for rel, expected in manifest['code_config_sha256'].items():
        if digest(ROOT/rel) != expected:
            errors.append('code or config changed: ' + rel)
    if verify_archives(out/'archives') != manifest['chart_snapshots']:
        errors.append('raw chart snapshot binding')
    expected = [name for name, _, _ in commands(out)] + ['core_tests', 'research_tests']
    if manifest['input_mode'] == 'official_download':
        expected.insert(0, 'download')
    if [row['name'] for row in manifest['stages']] != expected:
        errors.append('complete dependency order')
    for row in manifest['stages']:
        if row['returncode'] or row['status'] != 'passed' or digest(out/row['log']) != row['log_sha256']:
            errors.append('failed stage/log binding: ' + row['name'])
    canon = lambda x: hashlib.sha256(json.dumps(x, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode()).hexdigest()
    pins = [('a7', read(ROOT/'configs/support_sensitivity_a8.json')['source_a7_scientific_sha256']),
            ('a8', read(ROOT/'configs/support_optimization_a9.json')['source_a8_scientific_sha256']),
            ('a9', read(ROOT/'docs/support_optimization_a9_summary.json')['source_scientific_sha256'])]
    science = {}
    for name, expected in pins:
        x = read(out/name/'result.json')
        science[name] = canon({k: v for k, v in x.items() if k != 'total_computation_seconds'})
        if science[name] != expected:
            errors.append('frozen scientific result changed: ' + name)
    report = {'passed': not errors, 'errors': errors, 'stages_checked': len(manifest['stages']),
              'output_files_checked': len(result['output_sha256']), 'scientific_sha256': science,
              'acceptance_sha256': digest(out/'acceptance.json'),
              'strict_original_thesis_reproduction': False}
    args.json_out.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
