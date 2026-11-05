"""A10 must reject changed raw inputs and build an explicit fresh dependency graph."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import run_reproduction


class ReproductionEntryTests(unittest.TestCase):
    def test_archive_tamper_even_same_size_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); (root/'configs').mkdir(); (root/'raw').mkdir()
            data = b'fixed original archive'; (root/'raw/test.zip').write_bytes(data)
            (root/'configs/open_chart_long_beach_stage_a.json').write_text(json.dumps({'charts': [
                {'cell': 'test', 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(), 'url': 'test-only'}]}))
            with patch.object(run_reproduction, 'ROOT', root):
                self.assertIn('test.zip', run_reproduction.verify_archives(root/'raw'))
                (root/'raw/test.zip').write_bytes(b'X' + data[1:])
                with self.assertRaisesRegex(ValueError, 'snapshot missing or changed'):
                    run_reproduction.verify_archives(root/'raw')

    def test_all_generated_arguments_use_new_run_directory(self):
        output = Path('/tmp/new_reproduction_only')
        plan = run_reproduction.commands(output)
        names = [row[0] for row in plan]
        self.assertEqual(len(names), len(set(names)))
        for _, script, arguments in plan:
            self.assertTrue((run_reproduction.ROOT/script).is_file())
            for value in arguments:
                if isinstance(value, Path):
                    self.assertTrue(value.is_relative_to(output))
        self.assertLess(names.index('a8_audit'), names.index('a9_run'))
        self.assertLess(names.index('a6_repair_audit'), names.index('a7_run'))


if __name__ == '__main__':
    unittest.main()
