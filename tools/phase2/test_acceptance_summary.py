import copy
import unittest

from summarize_acceptance import assess


class SummaryTest(unittest.TestCase):
    def setUp(self):
        base = dict(passed=True, criteria_sha256='criteria', calibration_sha256='calibration',
                    evaluator_sha256='evaluator', geometry_sha256='geometry',
                    pipeline_source_sha256={'source':'hash'}, installed_adapter_sha256='binary',
                    confidence_checks={'low_texture':'passed', 'right_occluded':'passed'})
        self.runs = [dict(copy.deepcopy(base), scene_m=i) for i in (1, 2, 4)]
        self.loss = dict(passed=True, installed_adapter_sha256='binary')

    def test_one_unobserved_scene_requires_actual_observation_elsewhere(self):
        self.runs[0]['confidence_checks']['right_occluded'] = 'not_observed'
        self.assertEqual(assess(self.runs, self.loss), [])
        for r in self.runs:
            r['confidence_checks']['right_occluded'] = 'not_observed'
        self.assertTrue(assess(self.runs, self.loss))

    def test_cannot_combine_different_pipeline_or_criteria_revisions(self):
        for key in ('pipeline_source_sha256', 'installed_adapter_sha256', 'criteria_sha256'):
            runs = copy.deepcopy(self.runs)
            runs[0][key] = 'different'
            self.assertTrue(assess(runs, self.loss))

    def test_failed_run_confidence_or_loss_cannot_be_accepted(self):
        self.assertTrue(assess(self.runs, {'passed':False}))
        self.runs[0]['confidence_checks']['right_occluded'] = 'failed'
        self.assertTrue(assess(self.runs, self.loss))
        self.runs[0]['confidence_checks']['right_occluded'] = 'passed'
        self.runs[0]['passed'] = False
        self.assertTrue(assess(self.runs, self.loss))

    def test_loss_result_from_another_binary_cannot_be_reused(self):
        self.assertTrue(assess(self.runs, dict(passed=True, installed_adapter_sha256='old-binary')))


if __name__ == '__main__':
    unittest.main()
