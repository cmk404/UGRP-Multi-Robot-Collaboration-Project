import unittest
from harness.goals import infer_goal
from harness.executive import TaskExecutive

class SearchGoalResponsivenessTests(unittest.TestCase):
    def test_red_find_is_search_goal(self):
        for text in ('빨간 블럭 찾아줘','빨간 블록 찾아봐','red block find'):
            self.assertEqual(infer_goal(text).kind, 'SEARCH')
            self.assertEqual(infer_goal(text).subject, 'RED_BLOCK')
    def test_search_compiles_directly(self):
        g=infer_goal('빨간 블럭 찾아줘')
        self.assertEqual(TaskExecutive().plan(g,['search','track','approach','pick']), ['search'])

if __name__ == '__main__': unittest.main()
