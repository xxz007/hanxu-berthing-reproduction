import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RunnerTests(unittest.TestCase):
    def test_reference_run_and_reproducibility_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'run'
            result = subprocess.run([sys.executable,str(ROOT/'scripts/run_s1.py'),
                                     '--output-dir',str(output)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            metrics = json.loads((output/'metrics.json').read_text())
            self.assertEqual((metrics['leaf_count'],metrics['free_leaf_count'],metrics['blocked_leaf_count']), (10,9,1))
            self.assertEqual(metrics['leaf_area_sum'],64)
            self.assertEqual(metrics['interior_overlap_pairs'],0)
            self.assertTrue(metrics['adjacency_symmetric'])
            self.assertEqual([row['exact_los'] for row in metrics['los_results']], [True,False,False,False])
            self.assertTrue(metrics['los_results'][2]['endpoints_only_says_clear'])
            with (output/'leaves.csv').open() as stream:
                self.assertEqual(len(list(csv.DictReader(stream))),10)
            environment = json.loads((output/'environment.json').read_text())
            self.assertEqual(environment['third_party_dependencies'],[])
            for name,digest in environment['source_sha256'].items():
                self.assertEqual(hashlib.sha256((output/'code_snapshot'/name).read_bytes()).hexdigest(),digest)
            self.assertEqual((output/'config.json').read_bytes(),(ROOT/'configs/s1_teaching_map.json').read_bytes())
            for name in ['index.html','quadtree.svg','queries.json','adjacency.json','summary.md']:
                self.assertTrue((output/name).is_file())

    def test_existing_results_are_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'run'
            output.mkdir()
            sentinel=output/'metrics.json'
            sentinel.write_text('previous accepted result')
            result=subprocess.run([sys.executable,str(ROOT/'scripts/run_s1.py'),
                                   '--output-dir',str(output)],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertEqual(sentinel.read_text(),'previous accepted result')
            self.assertEqual(list(output.iterdir()),[sentinel])

    def test_unsupported_conventions_fail_before_output_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            directory=Path(directory)
            config=json.loads((ROOT/'configs/s1_teaching_map.json').read_text())
            config['conventions']['obstacle_contact']='free'
            source=directory/'config.json'
            source.write_text(json.dumps(config))
            output=directory/'run'
            result=subprocess.run([sys.executable,str(ROOT/'scripts/run_s1.py'),
                                   '--config',str(source),'--output-dir',str(output)],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('Unsupported S1',result.stderr)
            self.assertFalse(output.exists())

    def test_configured_depth_changes_actual_partition(self):
        with tempfile.TemporaryDirectory() as directory:
            directory=Path(directory)
            config=json.loads((ROOT/'configs/s1_teaching_map.json').read_text())
            config['map']['max_depth']=2
            source=directory/'config.json'
            source.write_text(json.dumps(config))
            output=directory/'run'
            result=subprocess.run([sys.executable,str(ROOT/'scripts/run_s1.py'),
                                   '--config',str(source),'--output-dir',str(output)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            metrics=json.loads((output/'metrics.json').read_text())
            self.assertEqual(metrics['leaf_count'],7)
            self.assertEqual(metrics['uniform_cells_at_same_depth'],16)


if __name__=='__main__':
    unittest.main()
