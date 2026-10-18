from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drone_control_pkg.controller_node import DroneControllerNode


@pytest.mark.parametrize('goal, expected', [
    ((0.5, -1.0), (0.5, -1.0)),
    ((8.0, -9.0), (2.0, -2.7)),
    ((-8.0, 9.0), (-2.0, 2.7)),
    ((2.0, 2.7), (2.0, 2.7)),
])
def test_goal_bounds(goal, expected):
    node = SimpleNamespace(goal_bounds=(-2.0, 2.0, -2.7, 2.7), get_logger=Mock())
    assert DroneControllerNode.constrain_goal(node, *goal) == expected


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_goal_is_rejected(value):
    node = SimpleNamespace(goal_bounds=(-2.0, 2.0, -2.7, 2.7), get_logger=Mock())
    with pytest.raises(ValueError):
        DroneControllerNode.constrain_goal(node, value, 0.0)
