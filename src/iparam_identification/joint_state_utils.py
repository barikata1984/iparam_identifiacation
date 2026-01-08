import numpy as np
from sensor_msgs.msg import JointState


def process_joint_state_msg(msg, target_joints):
    """
    Extracts joint positions and velocities from a JointState message,
    ordering them according to target_joints.
    Handles strict name matching and suffix matching (e.g. for namespaced joints).

    Args:
        msg (JointState): The received ROS message.
        target_joints (list[str]): List of desired joint names in order.

    Returns:
        tuple: (joint_positions, joint_velocities) as numpy arrays, or (None, None) if incomplete.
    """
    if len(msg.position) < len(target_joints):
        return None, None

    try:
        name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
        name_to_vel = {name: vel for name, vel in zip(msg.name, msg.velocity)}

        ordered_pos = []
        ordered_vel = []

        for target_name in target_joints:
            if target_name in name_to_pos:
                ordered_pos.append(name_to_pos[target_name])
                ordered_vel.append(name_to_vel[target_name])
                continue

            found = False
            for msg_name in name_to_pos:
                if msg_name.endswith(target_name):
                    ordered_pos.append(name_to_pos[msg_name])
                    ordered_vel.append(name_to_vel[msg_name])
                    found = True
                    break

            if not found:
                return None, None

        return np.array(ordered_pos), np.array(ordered_vel)

    except Exception:
        return None, None
