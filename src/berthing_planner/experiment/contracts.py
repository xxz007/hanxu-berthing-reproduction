"""Create a new A3 config for scientifically identical, regenerated A2 inputs.

Only three fixed top-level timing fields may vary in A2 result JSON. Scientific
values, task/path geometry, configuration, navigation and footprint files keep
their external pins. This does not relax the strict A3 runner or overwrite a
reference config. Uses only the standard library.
"""
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
VOLATILE_FIELDS = ('tree_seconds', 'primary_planning_seconds', 'total_experiment_seconds')
GEOMETRY_FILES = ('covered.geojson', 'missed.geojson', 'repeated.geojson', 'initial_missed.geojson')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(*, coverage_dir, reference_config, output_config):
    if output_config.exists():
        raise FileExistsError('Output config already exists; use a new file')
    reference = json.loads(reference_config.read_text())
    if reference.get('a2_volatile_result_fields') != list(VOLATILE_FIELDS):
        raise ValueError('Reference volatile rules must be exactly the fixed three A2 timing fields')
    geometry_pins = reference['source_a2_geometry_sha256']
    if set(geometry_pins) != set(GEOMETRY_FILES):
        raise ValueError('Reference must pin exactly the four A2 footprint geometry files')

    a2 = coverage_dir
    result_path, provenance_path = a2/'result.json', a2/'provenance.json'
    result = json.loads(result_path.read_text())
    if result.get('status') != 'geometric_coverage_passed':
        raise ValueError('An equivalent A2 input must have passed geometric coverage')
    observed_timing = {}
    for field in VOLATILE_FIELDS:
        value = result.get(field)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0):
            raise ValueError('A2 timing fields must exist and be finite nonnegative numbers')
        observed_timing[field] = value
    scientific = {key: value for key, value in result.items() if key not in VOLATILE_FIELDS}
    canonical = json.dumps(scientific, sort_keys=True, separators=(',', ':'),
                           ensure_ascii=False, allow_nan=False).encode('utf-8')
    scientific_hash = hashlib.sha256(canonical).hexdigest()
    if scientific_hash != reference['source_a2_scientific_result_sha256']:
        raise ValueError('A2 scientific result changed; only the three timing fields may differ')
    if digest(a2/'config.json') != reference['source_a2_config_sha256']:
        raise ValueError('A2 source configuration differs from its external pin')
    nav_hash = digest(a2/'navigation_grid.json')
    if nav_hash != reference['source_a2_navigation_grid_sha256']:
        raise ValueError('A2 navigation grid differs from its external pin')
    observed_geometry = {name: digest(a2/name) for name in GEOMETRY_FILES}
    if observed_geometry != geometry_pins:
        raise ValueError('A2 footprint geometry differs from its external pins')

    provenance = json.loads(provenance_path.read_text())
    result_hash = digest(result_path)
    if provenance.get('result_sha256') != result_hash:
        raise ValueError('A2 provenance result hash is not self-consistent')
    if provenance.get('geometry_sha256') != observed_geometry:
        raise ValueError('A2 provenance footprint hashes are not self-consistent')
    input_hashes = provenance.get('input_and_code_sha256')
    if not isinstance(input_hashes, dict):
        raise ValueError('A2 provenance is missing its input hash mapping')
    navigation_records = [value for key, value in input_hashes.items()
                          if key.replace('\\', '/').rsplit('/', 1)[-1] == 'navigation_grid.json']
    if navigation_records != [nav_hash]:
        raise ValueError('A2 provenance must identify exactly one self-consistent navigation grid hash')

    output = copy.deepcopy(reference)
    output['source_a2_result_sha256'] = result_hash
    output['source_a2_provenance_sha256'] = digest(provenance_path)
    output['equivalent_a2_input_refreeze'] = {
        'recorded_utc': datetime.now(timezone.utc).isoformat(),
        'method': 'Exact canonical scientific JSON and fixed configuration/navigation/footprint pins; timing metadata only may vary',
        'reference_config_sha256': digest(reference_config),
        'reference_source_a2_result_sha256': reference['source_a2_result_sha256'],
        'reference_source_a2_provenance_sha256': reference['source_a2_provenance_sha256'],
        'scientific_result_sha256': scientific_hash,
        'excluded_top_level_result_fields': list(VOLATILE_FIELDS),
        'observed_timing_metadata': observed_timing,
        'external_geometry_and_navigation_pins_verified': True,
        'source_provenance_self_consistent': True,
        'reference_config_modified': False}
    encoded = json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    output_config.parent.mkdir(parents=True, exist_ok=True)
    with output_config.open('x', encoding='utf-8') as handle:
        handle.write(encoded)
    print(json.dumps({'status': 'equivalent_scientific_inputs_frozen',
                      'output_config': output_config.name,
                      'scientific_result_sha256': scientific_hash}, ensure_ascii=False))
    return output
