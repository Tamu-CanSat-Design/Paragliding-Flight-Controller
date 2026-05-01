from platformio.test.result import TestCase, TestStatus
from platformio.test.runners.base import TestRunnerBase


class CustomTestRunner(TestRunnerBase):
    def stage_testing(self):
        super().stage_testing()
        if not self.test_suite.cases:
            self.test_suite.add_case(
                TestCase(
                    name=self.test_suite.test_name,
                    status=TestStatus.PASSED,
                    stdout=self._testing_output_buffer.strip() or None,
                )
            )
