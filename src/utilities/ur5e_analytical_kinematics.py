import numpy as np
from utilities.wrist_end_kinematics_utils import get_regressor_matrix


class UR5eAnalyticalKinematics:
    def __init__(self):
        # UR5e Modified DH Parameters
        # Reference: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
        # a [m], d [m], alpha [rad]
        self.dh_params = [
            {"a": 0, "d": 0.1625, "alpha": np.pi / 2, "offset": 0},  # Joint 1
            {"a": -0.425, "d": 0, "alpha": 0, "offset": 0},  # Joint 2
            {"a": -0.3922, "d": 0, "alpha": 0, "offset": 0},  # Joint 3
            {"a": 0, "d": 0.1333, "alpha": np.pi / 2, "offset": 0},  # Joint 4
            {"a": 0, "d": 0.0997, "alpha": -np.pi / 2, "offset": 0},  # Joint 5
            {"a": 0, "d": 0.0996, "alpha": 0, "offset": 0},  # Joint 6
        ]
        self.gravity = np.array([0, 0, -9.81])

    def _dh_transform(self, a, d, alpha, theta):
        """
        Calculate the transformation matrix using Modified DH convention.
        T_{i-1, i} = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # Row 1
        r11 = ct
        r12 = -st
        r13 = 0
        r14 = a

        # Row 2
        r21 = st * ca
        r22 = ct * ca
        r23 = -sa
        r24 = -sa * d

        # Row 3
        r31 = st * sa
        r32 = ct * sa
        r33 = ca
        r34 = ca * d

        T = np.array(
            [
                [r11, r12, r13, r14],
                [r21, r22, r23, r24],
                [r31, r32, r33, r34],
                [0, 0, 0, 1],
            ]
        )
        return T

    def forward_kinematics(self, q, dq, ddq):
        """
        Compute Forward Kinematics (Position, Velocity, Acceleration).
        Returns results in Base Frame.

        Args:
            q: Joint positions (6,)
            dq: Joint velocities (6,)
            ddq: Joint accelerations (6,)

        Returns:
            T_06: Homogeneous transformation matrix of EE (4x4)
            omega_06: Angular velocity of EE in Base Frame (3,)
            vel_06: Linear velocity of EE in Base Frame (3,)
            alpha_06: Angular acceleration of EE in Base Frame (3,)
            acc_06: Linear acceleration of EE in Base Frame (3,)
        """
        # Initialize Base State (Frame 0)
        T = np.eye(4)
        omega = np.zeros(3)  # Angular Velocity
        vel = np.zeros(3)  # Linear Velocity
        alpha = np.zeros(3)  # Angular Acceleration
        acc = np.zeros(3)  # Linear Acceleration (Kinematic)

        for i, custom_params in enumerate(self.dh_params):
            a = custom_params["a"]
            d = custom_params["d"]
            alpha_dh = custom_params["alpha"]
            theta = q[i] + custom_params["offset"]

            qd = dq[i]
            qdd = ddq[i]

            # Transformation from {i-1} to {i}
            T_i_prev = self._dh_transform(a, d, alpha_dh, theta)

            # Helper matrices
            R_prev_0 = T[:3, :3]  # Rotation of {i-1} w.r.t {0}
            P_i_prev = T_i_prev[:3, 3]  # Position of {i} in {i-1}

            # Update Global Transformation T_{0, i}
            T = T @ T_i_prev

            # Z-axis of joint {i} expressed in Base Frame {0}
            # In Modified DH, joint axis i is along Z axis of frame {i}
            z_axis_i = T[:3, 2]

            # --- Recursive Newton-Euler (Forward Pass) ---

            # 1. Angular Velocity propagation
            # omega_i = omega_{i-1} + z_i * qd_i
            omega_new = omega + z_axis_i * qd

            # 2. Angular Acceleration propagation
            # alpha_i = alpha_{i-1} + z_i * qdd_i + omega_{i-1} x (z_i * qd_i)
            alpha_new = alpha + z_axis_i * qdd + np.cross(omega, z_axis_i * qd)

            # 3. Linear Velocity propagation
            # v_i = v_{i-1} + omega_i x r_{i-1, i}
            # Note: In MDH with revolute joints, origin {i} is fixed in {i-1} structure,
            # effectively rigidly attached to link {i-1} regarding the joint rotation.
            r_vec = R_prev_0 @ P_i_prev  # Vector from {i-1} to {i} in Base Frame
            vel_new = vel + np.cross(omega_new, r_vec)

            # 4. Linear Acceleration propagation
            # a_i = a_{i-1} + alpha_{i-1} x r + omega_{i-1} x (omega_{i-1} x r)
            # Note: using omega (previous) and alpha (previous) for transport contribution
            # But the rotation is effectively happening at joint i?
            # Standard RNE propagates acceleration of the *link frame origin*.
            acc_new = (
                acc + np.cross(alpha, r_vec) + np.cross(omega, np.cross(omega, r_vec))
            )

            # Update state for next iteration
            omega = omega_new
            alpha = alpha_new
            vel = vel_new
            acc = acc_new

        # Compute Proper Acceleration (subtract gravity)
        lin_acc_proper = acc - self.gravity

        return T, omega, vel, alpha, lin_acc_proper


def build_ur5e_kinematics_func(method="standard"):
    """
    Builds and returns a kinematics_func compatible with ExcitedTrajectory.
    The returned function takes (q, dq, ddq) and returns the 6x10 regressor matrix
    for the end-effector load, with vectors expressed in the End-Effector Frame.

    Args:
        method (str): 'standard', 'twist', or 'matrix'
    """
    if method == "twist":
        kinematics = UR5eTwistKinematics()

        def kinematics_func(q, dq, ddq):
            # Forward Kinematics (Twist formulation returns vectors in Body Frame!)
            _, omega_body, _, alpha_body, acc_body = (
                kinematics.forward_kinematics_twist(q, dq, ddq)
            )
            return get_regressor_matrix(acc_body, omega_body, alpha_body)

        return kinematics_func

    if method == "matrix":
        kinematics = UR5eMatrixKinematics()

        def kinematics_func(q, dq, ddq):
            # Forward Kinematics (Matrix formulation returns vectors in Body Frame!)
            _, omega_body, _, alpha_body, acc_body = (
                kinematics.forward_kinematics_matrix(q, dq, ddq)
            )
            return get_regressor_matrix(acc_body, omega_body, alpha_body)

        return kinematics_func

    # Default standard method
    kinematics = UR5eAnalyticalKinematics()

    def kinematics_func(q, dq, ddq):
        # Forward Kinematics (returns vectors in Base Frame)
        T, omega_base, _, alpha_base, acc_base = kinematics.forward_kinematics(
            q, dq, ddq
        )

        # Rotate to End-Effector Frame (Body Frame)
        # T is T_{base, ee}
        R_base_ee = T[:3, :3]
        R_ee_base = R_base_ee.T

        omega_body = R_ee_base @ omega_base
        alpha_body = R_ee_base @ alpha_base
        acc_body = R_ee_base @ acc_base

        # Compute Regressor
        return get_regressor_matrix(acc_body, omega_body, alpha_body)

    return kinematics_func


class UR5eTwistKinematics(UR5eAnalyticalKinematics):
    """
    Kinematics implementation based on Modern Robotics (Lynch & Park), Chapter 8.
    Using Twist-Wrench formulation and Recursive Newton-Euler.
    """

    def _adjoint(self, T):
        """
        Adjoint map of SE(3).
        Ad_T = [[R, 0], [p_skew @ R, R]] for Twist = [omega, vel]
        """
        R = T[:3, :3]
        p = T[:3, 3]

        p_skew = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])

        adj = np.zeros((6, 6))
        adj[:3, :3] = R
        adj[3:, 3:] = R
        adj[3:, :3] = p_skew @ R
        return adj

    def _ad_operator(self, V):
        """
        Lie Bracket operator 'ad'.
        ad_V = [[omega_skew, 0], [v_skew, omega_skew]] for Twist V = [omega, v]
        """
        omega = V[:3]
        v = V[3:]

        omega_skew = np.array(
            [
                [0, -omega[2], omega[1]],
                [omega[2], 0, -omega[0]],
                [-omega[1], omega[0], 0],
            ]
        )
        v_skew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

        ad = np.zeros((6, 6))
        ad[:3, :3] = omega_skew
        ad[3:, :3] = v_skew
        ad[3:, 3:] = omega_skew
        return ad

    def forward_kinematics_twist(self, q, dq, ddq):
        """
        Compute Forward Dynamics (First Pass) using Twist formulation.
        MR Chapter 8.3, Algorithm 8.1 (ForwardNewtonEuler).

        Returns:
            T_0n: Pose of frame {n} (EE) in {0}
            V_n: Spatial Velocity of {n} in frame {n} (Body Twist)
            dV_n: Spatial Acceleration of {n} in frame {n} (Body Acceleration)
        """
        # Initialize Base State (Frame 0)
        # V_0 = [0, 0, 0, 0, 0, 0]
        # dV_0 = [0, 0, 0, 0, 0, -g] (Gravity as base acceleration)
        # However, to be consistent with "kinematic acceleration" + gravity later,
        # we start with 0 and subtract gravity at the end, or propagate gravity.
        # MR formulation usually propagates gravity as dV_0 = -g.

        # Let's stick to kinematic acceleration and return proper acceleration at the end.

        T = np.eye(4)
        V = np.zeros(6)
        dV = np.zeros(6)

        # We need dV_0 = [0,0,0, -R^T g] ??
        # Gravity is linear acceleration. In MR notation V = [w; v].
        # dV_0 = [0; -g_base].
        # But this dV is expressed in Frame {0}.
        # Let's include gravity in propagation.
        g_vec = -self.gravity  # +9.81 upwards
        dV[3:] = g_vec

        for i, custom_params in enumerate(self.dh_params):
            # Calculate M_{i, i-1} (Relative Pose at theta=0)
            a = custom_params["a"]
            d = custom_params["d"]
            alpha_dh = custom_params["alpha"]
            offset = custom_params["offset"]

            # The transformation T_{i-1, i} in MDH is:
            # Rot_x(alpha) Trans_x(a) Rot_z(theta) Trans_z(d)
            # This can be decomposed into:
            # M_relative = Rot_x(alpha) Trans_x(a) Trans_z(d)  (Static offset part?)
            # No, theta is in the middle.
            # T(theta) = A(fixed_part_1) * Rot_z(theta) * B(fixed_part_2)
            # T(theta) = T_i_prev computed in base class.

            theta = q[i] + offset
            dtheta = dq[i]
            ddtheta = ddq[i]

            # 1. Pose Propagation
            T_i_prev = self._dh_transform(a, d, alpha_dh, theta)
            # T_{i, i-1} = T_i_prev^{-1}
            # We need T_{i, i-1} to transform vectors from {i-1} to {i}.
            T_i_prev_inv = np.linalg.inv(T_i_prev)

            # Update Global T
            T = T @ T_i_prev

            # Adjoint of T_{i, i-1}
            Ad_T_inv = self._adjoint(T_i_prev_inv)

            # Joint Screw Axis in Frame {i}
            # In MDH, joint i rotates about Z_i.
            # So A_i = [0, 0, 1, 0, 0, 0] (Pure rotation about Z)
            Ai = np.zeros(6)
            Ai[2] = 1.0

            # 2. Twist Propagation
            # V_i = Ad_{T_{i,i-1}} (V_{i-1}) + A_i * dtheta
            V_new = Ad_T_inv @ V + Ai * dtheta

            # 3. Acceleration Propagation
            # dV_i = Ad_{T_{i,i-1}} (dV_{i-1}) + ad_{V_i} (A_i * dtheta) + A_i * ddtheta
            # Note: The term ad_{V_i} is Lie Bracket [V_i, Ai*dtheta]
            # MR Eq 8.52 uses V_{i} here?
            # Yes, V_i is the new twist.

            cross_term = self._ad_operator(V_new) @ (Ai * dtheta)
            dV_new = Ad_T_inv @ dV + cross_term + Ai * ddtheta

            V = V_new
            dV = dV_new

        # V and dV are now in End-Effector Frame (Body Frame).
        # V = [omega; v], dV = [domega; dv] (Contains gravity if we propagated dV0=-g)

        # We need "Proper Acceleration" for Force/Torque calculation.
        # Since we propagated gravity as base acceleration (upwards),
        # the resulting dV is effectively [alpha; a_proper].

        omega_body = V[:3]
        vel_body = V[3:]
        alpha_body = dV[:3]
        acc_body_proper = dV[3:]

        return T, omega_body, vel_body, alpha_body, acc_body_proper


class UR5eMatrixKinematics(UR5eTwistKinematics):
    """
    Closed-form (Matrix) Kinematics based on Modern Robotics (Lynch & Park).
    Equation 8.54 - 8.56.
    Avoids explicit loops for propagation by building a large matrix L.
    """

    def forward_kinematics_matrix(self, q, dq, ddq):
        """
        Compute Forward Dynamics using Matrix form.

        Returns:
            T_0n: Pose of frame {n} (EE) in {0}
            omega_body, vel_body: Body Twist of EE
            alpha_body, acc_body: Body Acceleration of EE (Proper)
        """
        n = len(self.dh_params)  # 6

        # 1. Precompute transforms and Adjoints
        # We need T_{i, i-1} for all i
        # And Ad_{T_{i, i-1}}

        # Recursion: V_i = Ad_{T_{i,i-1}} V_{i-1} + A_i dq_i
        # Let P_i = Ad_{T_{i, i-1}}.
        # V = L * (A * dq)
        # L = [[I, 0..], [P2, I, 0..], [P3P2, P3, I..] ...]

        # We need T_{i, i-1} = Inv( T_{i-1, i} )

        T_global = np.eye(4)

        # Store individual P_i matrices (6x6)
        Ps = []

        # Store A_i vectors (6x6 diagonal blocks later? No, just vectors)
        # In MDH, A_i is always [0,0,1,0,0,0] in frame {i}
        Ai_vec = np.zeros(6)
        Ai_vec[2] = 1.0

        # Construct inputs vectors
        # A_dq = [A1 dq1; A2 dq2 ...]
        A_dq = np.zeros((6 * n))
        A_ddq = np.zeros((6 * n))

        for i, custom_params in enumerate(self.dh_params):
            a = custom_params["a"]
            d = custom_params["d"]
            alpha_dh = custom_params["alpha"]
            offset = custom_params["offset"]

            theta = q[i] + offset

            # T_{i-1, i}
            T_i_prev = self._dh_transform(a, d, alpha_dh, theta)

            # T_{i, i-1}
            T_i_prev_inv = np.linalg.inv(T_i_prev)

            # P_i = Ad_{T_{i, i-1}}
            P_i = self._adjoint(T_i_prev_inv)
            Ps.append(P_i)

            # Update global pose
            T_global = T_global @ T_i_prev

            # Fill Input Vectors
            idx = i * 6
            A_dq[idx : idx + 6] = Ai_vec * dq[i]
            A_ddq[idx : idx + 6] = Ai_vec * ddq[i]

        # 2. Build L Matrix (6n x 6n)
        # L is Lower Triangular Block Matrix
        L = np.zeros((6 * n, 6 * n))

        # Diagonal blocks are Identity
        for i in range(n):
            idx = i * 6
            L[idx : idx + 6, idx : idx + 6] = np.eye(6)

        # Off-diagonal blocks
        # L_{i, j} = P_i * P_{i-1} * ... * P_{j+1}  (for i > j)
        # L_{i, i-1} = P_i
        # L_{i, i-2} = P_i * P_{i-1}
        # This can be computed cumulatively row by row.

        for i in range(1, n):
            row_idx = i * 6
            prev_row_idx = (i - 1) * 6

            # To compute row i blocks from row i-1 blocks:
            # L_{i, j} = P_i @ L_{i-1, j}  for j < i

            P_i = Ps[i]  # P corresponding to T_{i, i-1} (Wait, Ps[0] is T_{1,0}?)
            # i=0 (Joint 1): T_{1, 0}. Ps[0] = Ad_{T_{1,0}}. V_1 = P_0 V_0 + A1 dq1.
            # V0 is base velocity (usually 0).
            # So first row blocks are 0 except diagonal.

            # i=1 (Joint 2): V_2 = P_2 V_1 + A2 dq2.
            # V_2 = P_2 (P_1 V_0 + A1 dq1) + A2 dq2
            #     = (P_2 P_1) V_0 + P_2 A1 dq1 + A2 dq2.
            # The matrix L maps (A dq) to V. V0 is handled separately or included as dummy input.
            # Here we assume V0 = 0.

            # Block L_{i, j} maps j-th input to i-th output.
            # L_{i, j} = P_i @ L_{i-1, j}

            # Vectorized block multiplication for previous columns
            # L[row_idx:row_idx+6, :row_idx] = P_i @ L[prev_row_idx:prev_row_idx+6, :row_idx]
            # But L is sparse/blocky.

            # Let's just loop j < i
            for j in range(i):
                col_idx = j * 6
                L[row_idx : row_idx + 6, col_idx : col_idx + 6] = (
                    P_i @ L[prev_row_idx : prev_row_idx + 6, col_idx : col_idx + 6]
                )

        # 3. Compute All Twists V
        # V = L @ A_dq
        # Note: If base velocity V0 != 0, we need to add contribution L[:6n, :6] @ P_1 @ V0 ?
        # Actually V1 = P1 V0 + ...
        # V = L @ A_dq + [P1 V0; P2 P1 V0; ...]
        # Here V0=0.

        V_all = L @ A_dq

        # 4. Compute Coriolis Terms
        # ad_{V} (A dq)
        # We need a stacked vector of ad_{Vi} (Ai dqi)
        Coriolis_vec = np.zeros(6 * n)

        for i in range(n):
            idx = i * 6
            V_i = V_all[idx : idx + 6]
            A_dqi = A_dq[idx : idx + 6]  # This is Ai * dq[i]

            # ad_{Vi} * A_dqi = [V_i, A_dqi]
            # Use _ad_operator
            val = self._ad_operator(V_i) @ A_dqi
            Coriolis_vec[idx : idx + 6] = val

        # 5. Compute All Accelerations dV
        # dV = L @ (A_ddq + Coriolis)
        # + Gravity contribution?
        # dV_0 is base acceleration.
        # dV_1 = P_1 dV_0 + ...
        # Contribution of dV_0 to dV_i is (P_i ... P_1) dV_0.
        # This is exactly the first block column of L multiplied by P_1??
        # Actually L_{i, 0} = P_i ... P_2 P_1 ?? No.
        # In our code, loop for j starts at 0.
        # L_{i, 0} is P_i ... P_1 ? Let's check i=1.
        # L_{1, 0} = P_1 @ L_{0,0} = P_1 @ I = P_1. Correct.
        # So L_{i, 0} maps frame {0} input to frame {i}.
        # So providing dV_0 as input to "0-th channel" NO.
        # 0-th channel is A1 ddq1.
        # We need separate term for Base Acceleration.

        # Base Acceleration Term
        # dV_base_all = L[:, :6] @ (P_0 @ dV_0) ??
        # No.
        # dV_i = L_{i,0} (A1 ddq1) + ... + (Propagated dV_0).
        # Propagated dV_0 = (Ad_{T_{i,0}}) dV_0.
        # Ad_{T_{i,0}} = Ad_{T_{i,i-1}} ... Ad_{T_{1,0}}.
        # This is exactly L_{i, 0} @ P_0 ??
        # Wait, L_{i,0} corresponds to input at index 0 (Joint 1).
        # Joint 1 is "between" 0 and 1.
        # L_{i,0} corresponds to (A1 ddq1).
        # The base acceleration comes from "before" Joint 1.
        # Let's say dV_0 is input.
        # dV_1 = P_1 dV_0 + ...
        # dV_i = (P_i ... P_1) dV_0 + ...
        # And L_{i, 0} is indeed (P_i ... P_1).

        # So we can compute base acc contribution using the first block-column of L.

        g_vec = -self.gravity  # Upwards
        dV_0 = np.zeros(6)
        dV_0[3:] = g_vec

        # Base contribution for each link i: L_{i, 0} @ (P_1^{-1} @ P_1 @ dV_0) ??
        # No. L_{i,0} = P_i ... P_1.
        # So BaseContribution_i = L_{i,0} @ dV_0.

        # But wait, L is 6n x 6n.
        # L_{i,0} (block) is 6x6.
        # So we can just multiply L[:, :6] by dV_0.

        Base_contribution = L[:, :6] @ dV_0

        # Total dV
        dV_all = L @ (A_ddq + Coriolis_vec) + Base_contribution

        # Extract last link (n-1) values
        last_idx = (n - 1) * 6
        V_ee = V_all[last_idx : last_idx + 6]
        dV_ee = dV_all[last_idx : last_idx + 6]

        omega_body = V_ee[:3]
        vel_body = V_ee[3:]
        alpha_body = dV_ee[:3]
        acc_body_proper = dV_ee[3:]

        return T_global, omega_body, vel_body, alpha_body, acc_body_proper
