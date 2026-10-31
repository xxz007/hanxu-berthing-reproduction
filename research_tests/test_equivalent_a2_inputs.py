"""Small self-contained tests for timing-only A2 input equivalence."""
from contextlib import redirect_stdout
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from freeze_equivalent_a2_inputs import main


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class EquivalentA2InputsTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.a2 = self.root/'a2'
        self.a2.mkdir()
        self.result = {'status':'geometric_coverage_passed',
                       'path':[[0,0],[10,0]],
                       'legs':[{'kind':'sweep','scanning':True,'points':[[0,0],[10,0]],'length_m':10}],
                       'length_m':10,
                       'vector_coverage':{'target_area_m2':200,'covered_area_m2':199.8,'coverage_ratio':.999},
                       'tree_seconds':1.0,'primary_planning_seconds':2.0,'total_experiment_seconds':3.0}
        self.write_json(self.a2/'result.json',self.result)
        self.write_json(self.a2/'config.json',{'radius_m':50,'target':'完整目标'})
        self.write_json(self.a2/'navigation_grid.json',{'bounds':[0,0,20,20],'n':2,'rows_bottom_to_top':[[0,0],[1,1]]})
        names=('covered.geojson','missed.geojson','repeated.geojson','initial_missed.geojson')
        for name in names:
            self.write_json(self.a2/name,{'type':'FeatureCollection','features':[
                {'type':'Feature','properties':{'kind':name},'geometry':{'type':'Point','coordinates':[0,0]}}]})
        self.provenance = {'result_sha256':sha(self.a2/'result.json'),
                           'geometry_sha256':{name:sha(self.a2/name) for name in names},
                           'input_and_code_sha256':{'/different_machine/run/navigation_grid.json':sha(self.a2/'navigation_grid.json')}}
        self.write_json(self.a2/'provenance.json',self.provenance)
        scientific = {key:value for key,value in self.result.items()
                      if key not in ('tree_seconds','primary_planning_seconds','total_experiment_seconds')}
        scientific_hash = hashlib.sha256(json.dumps(scientific,sort_keys=True,separators=(',',':'),
                                                   ensure_ascii=False,allow_nan=False).encode()).hexdigest()
        self.reference = {'stage':'A3 fixture','source_a2_result_sha256':sha(self.a2/'result.json'),
                          'source_a2_provenance_sha256':sha(self.a2/'provenance.json'),
                          'source_a2_config_sha256':sha(self.a2/'config.json'),
                          'source_a2_scientific_result_sha256':scientific_hash,
                          'a2_volatile_result_fields':['tree_seconds','primary_planning_seconds','total_experiment_seconds'],
                          'source_a2_navigation_grid_sha256':sha(self.a2/'navigation_grid.json'),
                          'source_a2_geometry_sha256':self.provenance['geometry_sha256'],
                          'platform_speeds_mps':{'usv':3,'uuv_1':1.5,'uuv_2':1.5}}
        self.reference_path = self.root/'reference.json'
        self.write_json(self.reference_path,self.reference)
        self.output = self.root/'new_config.json'

    def tearDown(self):
        self.temporary.cleanup()

    def write_json(self,path,value):
        path.write_text(json.dumps(value,ensure_ascii=False)+'\n')

    def update_result(self,result,consistent_provenance=True):
        self.write_json(self.a2/'result.json',result)
        if consistent_provenance:
            self.provenance['result_sha256']=sha(self.a2/'result.json')
            self.write_json(self.a2/'provenance.json',self.provenance)

    def freeze(self):
        with redirect_stdout(io.StringIO()):
            return main(['--a2-run-dir',str(self.a2),'--reference-config',str(self.reference_path),
                         '--output-config',str(self.output)])

    def assert_rejected(self):
        with self.assertRaises(ValueError):
            self.freeze()
        self.assertFalse(self.output.exists())

    def test_changed_timing_only_accepts_new_hashes_without_touching_reference(self):
        reference_bytes=self.reference_path.read_bytes()
        result=copy.deepcopy(self.result)
        result.update(tree_seconds=.125,primary_planning_seconds=5,total_experiment_seconds=9.75)
        self.update_result(result)
        self.provenance['recorded_utc']='a later machine run'
        self.write_json(self.a2/'provenance.json',self.provenance)
        output=self.freeze()
        self.assertEqual(output['source_a2_result_sha256'],sha(self.a2/'result.json'))
        self.assertEqual(output['source_a2_provenance_sha256'],sha(self.a2/'provenance.json'))
        self.assertEqual(output['source_a2_scientific_result_sha256'],self.reference['source_a2_scientific_result_sha256'])
        self.assertEqual(self.reference_path.read_bytes(),reference_bytes)
        self.assertEqual(output['platform_speeds_mps'],self.reference['platform_speeds_mps'])
        self.assertTrue(output['equivalent_a2_input_refreeze']['source_provenance_self_consistent'])
        self.assertNotIn('/different_machine/',self.output.read_text())
        self.assertNotIn(str(self.root),self.output.read_text())

    def test_changed_scan_geometry_is_rejected_even_with_updated_provenance(self):
        result=copy.deepcopy(self.result)
        result['legs'][0]['points'][1]=[11,0]
        result['legs'][0]['length_m']=11
        result['path'][1]=[11,0]
        result['length_m']=11
        self.update_result(result)
        self.assert_rejected()

    def test_changed_scientific_coverage_metric_is_rejected(self):
        result=copy.deepcopy(self.result)
        result['vector_coverage'].update(covered_area_m2=200,coverage_ratio=1.0)
        self.update_result(result)
        self.assert_rejected()

    def test_changed_navigation_is_rejected_even_with_updated_provenance(self):
        self.write_json(self.a2/'navigation_grid.json',{'bounds':[0,0,20,20],'n':2,'rows_bottom_to_top':[[0,0],[0,1]]})
        self.provenance['input_and_code_sha256']['/different_machine/run/navigation_grid.json']=sha(self.a2/'navigation_grid.json')
        self.write_json(self.a2/'provenance.json',self.provenance)
        self.assert_rejected()

    def test_changed_geometry_is_rejected_even_with_updated_provenance(self):
        self.write_json(self.a2/'covered.geojson',{'type':'Point','coordinates':[1,1]})
        self.provenance['geometry_sha256']['covered.geojson']=sha(self.a2/'covered.geojson')
        self.write_json(self.a2/'provenance.json',self.provenance)
        self.assert_rejected()

    def test_changed_source_config_is_rejected(self):
        self.write_json(self.a2/'config.json',{'radius_m':100,'target':'完整目标'})
        self.assert_rejected()

    def test_stale_result_provenance_is_rejected(self):
        result=copy.deepcopy(self.result)
        result['tree_seconds']=7
        self.update_result(result,consistent_provenance=False)
        self.assert_rejected()

    def test_provenance_navigation_and_geometry_records_must_be_self_consistent(self):
        original=copy.deepcopy(self.provenance)
        mutations=[
            {'input_and_code_sha256':{'/different_machine/run/navigation_grid.json':'0'*64}},
            {'input_and_code_sha256':{'a/navigation_grid.json':self.reference['source_a2_navigation_grid_sha256'],
                                     'b/navigation_grid.json':self.reference['source_a2_navigation_grid_sha256']}},
            {'geometry_sha256':{'covered.geojson':'0'*64}}]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                changed=copy.deepcopy(original);changed.update(mutation)
                self.write_json(self.a2/'provenance.json',changed)
                self.assert_rejected()

    def test_timing_fields_must_be_present_finite_nonnegative_real_numbers(self):
        for value in (-1,math.nan,math.inf,True,'3.0',None):
            with self.subTest(value=value):
                result=copy.deepcopy(self.result);result['tree_seconds']=value
                self.update_result(result);self.assert_rejected()
        result=copy.deepcopy(self.result);result.pop('tree_seconds')
        self.update_result(result);self.assert_rejected()

    def test_nonfinite_science_cannot_be_hidden_by_timing_exclusion(self):
        result=copy.deepcopy(self.result);result['vector_coverage']['coverage_ratio']=math.nan
        self.update_result(result);self.assert_rejected()

    def test_volatile_rules_cannot_be_extended_replaced_or_removed(self):
        lists=[self.reference['a2_volatile_result_fields']+['length_m'],
               ['tree_seconds','primary_planning_seconds','length_m'],
               ['tree_seconds','primary_planning_seconds']]
        for fields in lists:
            with self.subTest(fields=fields):
                ref=copy.deepcopy(self.reference);ref['a2_volatile_result_fields']=fields
                self.write_json(self.reference_path,ref);self.assert_rejected()

    def test_nested_field_with_a_timing_name_is_still_scientific(self):
        result=copy.deepcopy(self.result);result['legs'][0]['tree_seconds']=1
        self.update_result(result);self.assert_rejected()

    def test_reference_cannot_drop_a_geometry_pin(self):
        ref=copy.deepcopy(self.reference);ref['source_a2_geometry_sha256'].pop('missed.geojson')
        self.write_json(self.reference_path,ref);self.assert_rejected()

    def test_existing_output_and_reference_are_never_overwritten(self):
        self.output.write_text('sentinel')
        with self.assertRaises(FileExistsError):
            self.freeze()
        self.assertEqual(self.output.read_text(),'sentinel')
        before=self.reference_path.read_bytes()
        with self.assertRaises(FileExistsError),redirect_stdout(io.StringIO()):
            main(['--a2-run-dir',str(self.a2),'--reference-config',str(self.reference_path),
                  '--output-config',str(self.reference_path)])
        self.assertEqual(self.reference_path.read_bytes(),before)


if __name__=='__main__':
    unittest.main()
