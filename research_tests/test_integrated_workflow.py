"""Entry-boundary regressions for immutable fixed-case experiment runs."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.experiment.workflow import run, validate_config, verify_archives


class IntegratedBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((ROOT/'configs/experiment.json').read_text())

    def test_rejects_repeated_or_missing_registered_condition(self):
        for edit in ('duplicate','missing'):
            cfg=copy.deepcopy(self.cfg)
            if edit=='duplicate':cfg['control']['design']['case_order'].append('nominal')
            else:cfg['control']['cases'].pop('sensor_5m')
            with self.assertRaisesRegex(ValueError,'unique, complete'):
                validate_config(cfg)

    def test_rejects_mixed_nominal_period_before_creating_output(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);cfg=copy.deepcopy(self.cfg);cfg['control']['step_s']=1.
            (root/'config.json').write_text(json.dumps(cfg))
            with self.assertRaisesRegex(ValueError,'same 0.5'):
                run(config=root/'config.json',output_dir=root/'result')
            self.assertFalse((root/'result').exists())

    def test_changed_archive_fails_without_output_or_network(self):
        with tempfile.TemporaryDirectory() as folder, patch('urllib.request.urlopen') as network:
            root=Path(folder);(root/'US5LGBCE.zip').write_bytes(b'changed snapshot')
            with self.assertRaisesRegex(ValueError,'snapshot missing or changed'):
                run(config=ROOT/'configs/experiment.json',output_dir=root/'result',archives_dir=root)
            self.assertFalse((root/'result').exists());network.assert_not_called()

    def test_verify_uses_both_size_and_content(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);path=root/'demo.zip';path.write_bytes(b'abc')
            cfg={'charts':[dict(cell='demo',bytes=3,sha256=hashlib.sha256(b'abc').hexdigest(),url='https://example.invalid/demo.zip')]}
            self.assertIn('demo.zip',verify_archives(root,cfg))
            path.write_bytes(b'abd')
            with self.assertRaises(ValueError):verify_archives(root,cfg)

    def test_default_package_has_no_stage_cli_dependency(self):
        # The final entry must remain independently callable after users move
        # reference CLIs away; a renamed subprocess chain would violate this.
        for folder in ('experiment','validation'):
            for path in (ROOT/'src/berthing_planner'/folder).glob('*.py'):
                for node in ast.walk(ast.parse(path.read_text())):
                    if isinstance(node,ast.ImportFrom):
                        self.assertFalse((node.module or '').startswith(('scripts','research_tests','run_support_','audit_support_')),str(path))
                    if isinstance(node,ast.Attribute) and isinstance(node.value,ast.Name) and node.value.id=='sys':
                        self.assertNotEqual(node.attr,'argv',str(path))


if __name__=='__main__':unittest.main()
