import copy
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_closed_loop_reproduction import completion_gate


class ClosedLoopGateTests(unittest.TestCase):
    def fixture(self):
        audits=[{'passed':True,'mission_constraints_certified':True}for _ in range(3)]
        names=('nominal','initial_offset','drift_005','sensor_010','combined','sensor_5m','current_over_limit')
        study={'passed':True,'cases':[{'case':n,'audit_passed':True,'constraint_outcome':'witnessed_violation'if n in('sensor_5m','current_over_limit')else'certified_pass'}for n in names]}
        return audits,study

    def test_audited_adverse_failure_does_not_hide_nominal_success(self):
        self.assertTrue(completion_gate(*self.fixture()))

    def test_failed_mathematical_audit_blocks_completion(self):
        a,s=self.fixture();s['cases'][3]['audit_passed']=False
        self.assertFalse(completion_gate(a,s))

    def test_no_nominal_pass_blocks_completion(self):
        a,s=self.fixture();a[0]['mission_constraints_certified']=False
        self.assertFalse(completion_gate(a,s))

    def test_cannot_claim_every_stress_case_passed(self):
        a,s=self.fixture();s['cases'][-1]['constraint_outcome']='certified_pass'
        self.assertFalse(completion_gate(a,s))


if __name__=='__main__':unittest.main()
