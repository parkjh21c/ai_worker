"""Right gripper joint <-> fingertip opening, from the measured table."""
import numpy as np
import yaml


def load_table(path):
    """[(joint rad, opening m)] sorted by joint; opening shrinks as the joint closes."""
    with open(path) as f:
        rows = sorted((float(j), float(o)) for j, o in yaml.safe_load(f)['table'])
    if len(rows) < 2:
        raise ValueError(f'{path}: need at least 2 rows')
    return rows


def joint_for(table, opening):
    """Gripper joint for an opening (m), linear between measured rows, clipped to the table."""
    joints, openings = zip(*table)
    return float(np.interp(opening, openings[::-1], joints[::-1]))
