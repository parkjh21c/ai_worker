import math
import unittest

from ffw_pick.pick_moveit import search_tiers
import numpy as np


def yaw(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class TestSearchTiers(unittest.TestCase):

    def test_upright_by_turn_then_rolled_in_given_order(self):
        options = [{'rotation': yaw(40), 'roll': 0.0}, {'rotation': yaw(10), 'roll': 0.0},
                   {'rotation': yaw(5), 'roll': -0.2}, {'rotation': yaw(60), 'roll': 0.2}]
        upright, rolled = search_tiers(options, np.eye(3))
        turns = [e[0] for e in upright]
        self.assertEqual({e[1] for e in upright}, {0, 1})
        self.assertEqual(turns, sorted(turns))
        self.assertEqual([e[1] for e in upright][0], 1)
        # rolled keep ffw_grasp's order, both yaws each
        self.assertEqual([(e[1], e[2]) for e in rolled], [(2, False), (2, True), (3, False),
                                                          (3, True)])
        # old ffw_grasp rows without 'roll' are upright
        self.assertEqual(len(search_tiers([{'rotation': yaw(0)}], np.eye(3))[0]), 2)


if __name__ == '__main__':
    unittest.main()
