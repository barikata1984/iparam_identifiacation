#!/usr/bin/env python3
"""Verify that non-UR joint_states messages are correctly filtered."""

import sys
from pathlib import Path

import pytest

SRC_DIR = Path(__file__).resolve().parent.parent / "src"
sys.path.insert(0, str(SRC_DIR))

from utilities.tool0_kinematics import JOINT_ORDER

# Replicate the filter condition from tool0_kinematics_node.py
_UR_JOINT_NAMES = set(JOINT_ORDER)


def _would_be_accepted(msg_names: list[str]) -> bool:
    """Simulate the filter: returns True if the message would be processed."""
    return _UR_JOINT_NAMES.issubset(msg_names)


# --- UR driver messages (should be ACCEPTED) ---


def test_accept_ur_standard_order():
    """UR driver with standard joint order."""
    names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    assert _would_be_accepted(names)


def test_accept_ur_alphabetical_order():
    """UR driver sometimes publishes in alphabetical order."""
    names = [
        "elbow_joint",
        "shoulder_lift_joint",
        "shoulder_pan_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    assert _would_be_accepted(names)


def test_accept_ur_with_extra_joints():
    """UR message with additional joints (e.g. tool joint) should still pass."""
    names = JOINT_ORDER + ["tool_joint"]
    assert _would_be_accepted(names)


# --- Non-UR messages (should be REJECTED) ---


def test_reject_robotiq_gripper():
    """Robotiq gripper publishes single finger_joint."""
    names = ["finger_joint"]
    assert not _would_be_accepted(names)


def test_reject_robotiq_with_prefix():
    """Robotiq gripper with prefix."""
    names = ["a_bot_finger_joint"]
    assert not _would_be_accepted(names)


def test_reject_dynamixel():
    """Dynamixel leader arm joints."""
    names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    assert not _would_be_accepted(names)


def test_reject_empty():
    """Empty joint names."""
    assert not _would_be_accepted([])


def test_reject_partial_ur():
    """Only some UR joints present — incomplete message."""
    names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"]
    assert not _would_be_accepted(names)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
