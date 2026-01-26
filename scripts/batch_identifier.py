#!/usr/bin/env python3
import rospy
import message_filters
import numpy as np
import json
import os
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, Vector3
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Bool
from identifiers.tls import solve_tls_compare, print_tls_comparison
import matplotlib.pyplot as plt
import datetime


class BatchIdentifierNode:
    def __init__(self):
        rospy.init_node("batch_identifier", anonymous=True)

        self.wrench_topic = rospy.get_param("~wrench_topic", "/wrench")

        # Buffer for recorded frames
        # Each frame is a dict containing synced data
        self.recorded_frames = []
        self.is_recording = False

        # Subscribers
        self.sub_joint = message_filters.Subscriber("/joint_states", JointState)
        self.sub_wrench = message_filters.Subscriber(self.wrench_topic, WrenchStamped)

        # Subscribers for kinematics from wrist_end_kinematics_node
        self.sub_lv = message_filters.Subscriber(
            "/wrist_end_kinematics/lv_tool0", Vector3
        )
        self.sub_av = message_filters.Subscriber(
            "/wrist_end_kinematics/av_tool0", Vector3
        )
        self.sub_la = message_filters.Subscriber(
            "/wrist_end_kinematics/la_tool0", Vector3
        )
        self.sub_aa = message_filters.Subscriber(
            "/wrist_end_kinematics/aa_tool0", Vector3
        )
        self.sub_regressor = message_filters.Subscriber(
            "/wrist_end_kinematics/regressor", Float64MultiArray
        )

        # Synchronizer
        # We use ApproximateTimeSynchronizer because messages come from different sources
        queue_size = 100
        slop = 0.01  # 10ms slop
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [
                self.sub_joint,
                self.sub_wrench,
                self.sub_lv,
                self.sub_av,
                self.sub_la,
                self.sub_aa,
                self.sub_regressor,
            ],
            queue_size,
            slop,
            allow_headerless=True,
        )
        self.ts.registerCallback(self.callback)

        # Publisher for inertia parameters (10 elements)
        self.inertia_pub = rospy.Publisher(
            "~inertia_params", Float64MultiArray, queue_size=1, latch=True
        )

        # Publisher for identification status
        self.identified_pub = rospy.Publisher(
            "~iparams_identified", Bool, queue_size=1, latch=True
        )

        # Parameter names for display
        self.param_names = [
            "m",
            "mcx",
            "mcy",
            "mcz",
            "Ixx",
            "Iyy",
            "Izz",
            "Ixy",
            "Iyz",
            "Izx",
        ]

        rospy.loginfo("Batch Identifier Node Initialized.")
        rospy.loginfo(
            f"Subscribing to {self.wrench_topic} and /wrist_end_kinematics/..."
        )
        rospy.loginfo("Publishing inertia params to: ~inertia_params")
        rospy.loginfo("Publishing identification status to: ~iparams_identified")

    def callback(self, joint_msg, wrench_msg, lv_msg, av_msg, la_msg, aa_msg, reg_msg):
        if not self.is_recording:
            return

        # Parse data
        frame = {}
        frame["time"] = (
            rospy.Time.now().to_sec()
        )  # Or use message timestamp? Let's use joint_msg stamp
        frame["time"] = joint_msg.header.stamp.to_sec()

        # Joint State
        frame["joint_position"] = list(joint_msg.position)
        frame["joint_velocity"] = list(joint_msg.velocity)

        # Tool Kinematics
        frame["tool0_kinematics"] = {
            "lv": [lv_msg.x, lv_msg.y, lv_msg.z],
            "av": [av_msg.x, av_msg.y, av_msg.z],
            "la": [la_msg.x, la_msg.y, la_msg.z],
            "aa": [aa_msg.x, aa_msg.y, aa_msg.z],
        }

        # Regressor
        # Regressor is flattened 6x10
        reg_flat = np.array(reg_msg.data)
        reg_reshaped = reg_flat.reshape(6, 10)
        frame["regressor"] = reg_reshaped.tolist()

        # Wrench
        # Force/Torque from sensor
        # Invert wrench (sensor frame is reaction force)
        f = [
            -wrench_msg.wrench.force.x,
            -wrench_msg.wrench.force.y,
            -wrench_msg.wrench.force.z,
        ]
        n = [
            -wrench_msg.wrench.torque.x,
            -wrench_msg.wrench.torque.y,
            -wrench_msg.wrench.torque.z,
        ]
        frame["wrench"] = f + n  # [fx, fy, fz, nx, ny, nz]

        self.recorded_frames.append(frame)

    def plot_wrench_data(self, wrench_data, results_dir):
        """Plot F/T data with each component as separate series.

        Uses the same style as friction_identifier (alternating background stripes).
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        n_points = len(wrench_data)
        indices = np.arange(n_points)

        # Extract force and torque components
        wrench = np.array(wrench_data)
        force = wrench[:, :3]  # Fx, Fy, Fz
        torque = wrench[:, 3:]  # Tx, Ty, Tz

        # Alternating background stripes (5 sets = 10 stripes)
        stripe_width = n_points / 10
        for ax in [ax1, ax2]:
            for i in range(10):
                if i % 2 == 1:
                    ax.axvspan(
                        i * stripe_width,
                        (i + 1) * stripe_width,
                        alpha=0.15,
                        color="lightblue",
                        zorder=0,
                    )

        # Plot force components
        ax1.scatter(indices, force[:, 0], c="red", s=8, alpha=0.7, label="Fx")
        ax1.scatter(indices, force[:, 1], c="green", s=8, alpha=0.7, label="Fy")
        ax1.scatter(indices, force[:, 2], c="blue", s=8, alpha=0.7, label="Fz")
        ax1.set_ylabel("Force (N)", fontsize=10)
        ax1.set_title("Force", fontsize=12)
        ax1.legend(loc="upper right", fontsize=8)
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim(0, n_points)

        # Plot torque components
        ax2.scatter(indices, torque[:, 0], c="red", s=8, alpha=0.7, label="Tx")
        ax2.scatter(indices, torque[:, 1], c="green", s=8, alpha=0.7, label="Ty")
        ax2.scatter(indices, torque[:, 2], c="blue", s=8, alpha=0.7, label="Tz")
        ax2.set_xlabel("Sample Index", fontsize=10)
        ax2.set_ylabel("Torque (Nm)", fontsize=10)
        ax2.set_title("Torque", fontsize=12)
        ax2.legend(loc="upper right", fontsize=8)
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim(0, n_points)

        plt.tight_layout()

        # Save plot
        plot_path = os.path.join(results_dir, "wrench_scatter.png")
        fig.savefig(plot_path, dpi=150, bbox_inches="tight")
        rospy.loginfo(f"Saved wrench plot: {plot_path}")

        plt.show(block=False)
        plt.pause(0.5)

        return fig

    def get_range_input(self):
        """Get lower and upper bound percentages from user."""
        while True:
            try:
                print("\n" + "=" * 60)
                user_input = input(
                    "Enter lower and upper bounds (format: lower upper) > "
                ).strip()
                print("=" * 60)

                parts = user_input.split()
                if len(parts) != 2:
                    print(
                        "Invalid format. Please enter two numbers separated by space."
                    )
                    continue

                lower = float(parts[0])
                upper = float(parts[1])

                if not (0 <= lower < upper <= 100):
                    print("Invalid range. Lower must be < upper, both in [0, 100].")
                    continue

                return lower, upper

            except ValueError:
                print("Invalid input. Please enter numeric values.")

    def publish_inertia_params(self, params):
        """Publish inertia parameters as Float64MultiArray and set identified flag."""
        msg = Float64MultiArray()
        msg.data = params.tolist()
        self.inertia_pub.publish(msg)

        # Publish identification status
        self.identified_pub.publish(Bool(True))

        rospy.loginfo("Published inertia parameters")
        rospy.loginfo("Published iparams_identified = True")

    def run_cli(self):
        print("Batch Identifier CLI")
        print("--------------------")

        while not rospy.is_shutdown():
            input("Press [Enter] to START recording...")
            if rospy.is_shutdown():
                break

            self.recorded_frames = []
            self.is_recording = True
            print("RECORDING... Press [Enter] to STOP.")

            input()  # Wait for Enter to stop
            self.is_recording = False
            print(f"Stopped. Captured {len(self.recorded_frames)} frames.")

            if len(self.recorded_frames) == 0:
                print("No data recorded.")
                continue

            # Process data with interactive workflow
            accepted = self.process_data_interactive()

            if accepted:
                print("Inertia parameters published. Node continues running.")
                print("Press Ctrl+C to exit.")
                rospy.spin()
                break
            else:
                print("Ready for next batch.")

    def process_data_interactive(self):
        """Interactive data processing with plot, range selection, and confirmation.

        Returns:
            True if user accepted and params were published, False otherwise
        """
        import rospkg

        print("Processing data...")

        # Prepare results directory
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("iparam_identification")
        timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        dirname = f"batch_id_{timestamp_str}"
        results_dir = os.path.join(package_path, "results", dirname)
        os.makedirs(results_dir, exist_ok=True)

        # Prepare data
        start_time = self.recorded_frames[0]["time"]
        export_frames = []
        wrench_data = []

        for frame in self.recorded_frames:
            rel_time = frame["time"] - start_time
            ex_frame = frame.copy()
            ex_frame["time"] = rel_time
            export_frames.append(ex_frame)
            wrench_data.append(frame["wrench"])

        # Plot wrench data with friction_identifier style
        fig = self.plot_wrench_data(wrench_data, results_dir)

        # Get range from user
        print("\n" + "=" * 60)
        print("Specify the data range for inertia parameter regression")
        lower_pct, upper_pct = self.get_range_input()

        # Extract subset of data within range
        n_frames = len(export_frames)
        lower_idx = int(n_frames * lower_pct / 100)
        upper_idx = int(n_frames * upper_pct / 100)

        subset_frames = export_frames[lower_idx:upper_idx]

        if len(subset_frames) < 2:
            print("Not enough data points in selected range.")
            plt.close(fig)
            return False

        print(f"Using {len(subset_frames)} frames ({lower_pct}% - {upper_pct}%)")

        # Build regressor and wrench matrices from subset
        S_list = []
        W_list = []
        for frame in subset_frames:
            S_list.append(np.array(frame["regressor"]))
            W_list.append(np.array(frame["wrench"]))

        S_total = np.vstack(S_list)  # (6*N, 10)
        W_total = np.hstack(W_list)  # (6*N,)

        print(f"Constructed S matrix shape: {S_total.shape}")
        print(f"Constructed W vector shape: {W_total.shape}")

        # Solve OLS
        print("Solving OLS...")
        try:
            pi_ols, _, rank, _ = np.linalg.lstsq(S_total, W_total, rcond=None)
        except Exception as e:
            print(f"OLS Failed: {e}")
            pi_ols = np.zeros(10)
            rank = 0

        # Solve TLS
        print("Solving TLS...")
        tls_results = solve_tls_compare(S_total, W_total)

        # Get best TLS result (column scaling preferred)
        if tls_results.get("column") is not None:
            pi_tls = tls_results["column"].x
        elif tls_results.get("none") is not None:
            pi_tls = tls_results["none"].x
        else:
            pi_tls = np.zeros(10)

        plt.close(fig)

        # Show results
        print("\n" + "=" * 60)
        print("INERTIA PARAMETER ESTIMATION RESULTS")
        print("=" * 60)
        print(
            f"\nData range: {lower_pct}% - {upper_pct}% ({len(subset_frames)} frames)"
        )
        print(f"Matrix rank: {rank}")
        print("\n{:10s} {:>15s} {:>15s}".format("Param", "OLS", "TLS"))
        print("-" * 42)
        for i, name in enumerate(self.param_names):
            print(f"{name:10s} {pi_ols[i]:>15.6f} {pi_tls[i]:>15.6f}")
        print("=" * 60)
        print(f"\nTopic to publish: /batch_identifier/inertia_params")
        print("(Using TLS column-scaling result)")
        print("=" * 60)

        # Ask for confirmation
        while True:
            response = (
                input("\nAccept these results and publish? (y/n) > ").strip().lower()
            )
            if response in ["y", "yes"]:
                # Save results
                output_data = {
                    "meta": {
                        "timestamp": str(rospy.Time.now()),
                        "count": len(subset_frames),
                        "range_lower_pct": lower_pct,
                        "range_upper_pct": upper_pct,
                    },
                    "results": {
                        "ols": {"params": pi_ols.tolist()},
                        "tls": {"params": pi_tls.tolist()},
                    },
                    "frames": subset_frames,
                }

                save_path = os.path.join(results_dir, "result.json")
                with open(save_path, "w") as f:
                    json.dump(output_data, f, indent=2)
                print(f"Saved results to {save_path}")

                # Also save old-style plots
                self.plot_results(subset_frames, results_dir)

                # Publish parameters
                self.publish_inertia_params(pi_tls)
                return True

            elif response in ["n", "no"]:
                print("Results not accepted.")
                return False
            else:
                print("Please enter 'y' or 'n'.")

    def process_data(self):
        print("Processing data...")

        # Prepare matrices for Solver
        # Y = W (nx1 vector if stacked, or nx6)
        # linear system: W = S * pi
        # Collect all S and all W

        S_list = []
        W_list = []

        start_time = self.recorded_frames[0]["time"]

        # Organize data for JSON export structure (frames list is already self.recorded_frames)
        # But we need to make sure timestamps are relative if desired?
        # User example showed 0.0, 0.002... so likely relative time.

        export_frames = []

        for frame in self.recorded_frames:
            # Shift time
            rel_time = frame["time"] - start_time

            # Append to solving matrices
            S_list.append(np.array(frame["regressor"]))  # 6x10
            W_list.append(np.array(frame["wrench"]))  # 6

            # Create export frame copy with relative time
            ex_frame = frame.copy()
            ex_frame["time"] = rel_time
            export_frames.append(ex_frame)

        # Stack matrices
        # S_total: (6*N, 10)
        # W_total: (6*N, )
        S_total = np.vstack(S_list)
        W_total = np.hstack(W_list)  # Flattened wrench vector

        print(f"Constructed S matrix shape: {S_total.shape}")
        print(f"Constructed W vector shape: {W_total.shape}")

        # OLS
        print("Solving OLS...")
        try:
            # x, residuals, rank, s
            pi_ols, _, rank, _ = np.linalg.lstsq(S_total, W_total, rcond=None)
            print("OLS Result:")
            print(pi_ols)
            print(f"Rank: {rank}")
        except Exception as e:
            print(f"OLS Failed: {e}")
            pi_ols = np.zeros(10)

        # TLS with multiple scaling modes for comparison
        print("Solving TLS with different scaling modes...")
        tls_results = solve_tls_compare(S_total, W_total)

        # Print comparison
        param_names = [
            "m",
            "mcx",
            "mcy",
            "mcz",
            "Ixx",
            "Iyy",
            "Izz",
            "Ixy",
            "Iyz",
            "Izx",
        ]
        print_tls_comparison(tls_results, param_names)

        # Use COLUMN_ONLY as the primary TLS result
        if tls_results.get("column") is not None:
            pi_tls = tls_results["column"].x
            s_min = tls_results["column"].sigma_min
            print("TLS (column scaling) Result:")
            print(pi_tls)
            print(f"Min Singular Value: {s_min}")
        else:
            print("TLS (column scaling) Failed, trying no scaling...")
            if tls_results.get("none") is not None:
                pi_tls = tls_results["none"].x
                s_min = tls_results["none"].sigma_min
            else:
                print("All TLS methods failed.")
                pi_tls = np.zeros(10)
                s_min = 0.0

        # Export JSON
        output_data = {
            "meta": {"timestamp": str(rospy.Time.now()), "count": len(export_frames)},
            "results": {
                "ols": {"params": pi_ols.tolist()},
                "tls": {"params": pi_tls.tolist()},
            },
            "frames": export_frames,
        }

        filename = f"batch_ident_{int(rospy.Time.now().to_sec())}.json"

        # Save to package results directory
        # Prepare results directory
        import rospkg

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("iparam_identification")

        timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        dirname = f"batch_id_{timestamp_str}"
        results_dir = os.path.join(package_path, "results", dirname)

        if not os.path.exists(results_dir):
            os.makedirs(results_dir)

        # Save result.json
        save_path = os.path.join(results_dir, "result.json")
        with open(save_path, "w") as f:
            json.dump(output_data, f, indent=2)

        print(f"Saved results to {save_path}")

        # Plotting
        self.plot_results(export_frames, results_dir)

    def plot_results(self, frames, save_dir):
        times = [f["time"] for f in frames]

        # Extract data
        lv = np.array([f["tool0_kinematics"]["lv"] for f in frames])
        av = np.array([f["tool0_kinematics"]["av"] for f in frames])
        la = np.array([f["tool0_kinematics"]["la"] for f in frames])
        aa = np.array([f["tool0_kinematics"]["aa"] for f in frames])
        wrench = np.array([f["wrench"] for f in frames])

        # 1. Velocity Plot (lv, av)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        ax1.plot(times, lv[:, 0], label="x")
        ax1.plot(times, lv[:, 1], label="y")
        ax1.plot(times, lv[:, 2], label="z")
        ax1.set_title("Linear Velocity (lv)")
        ax1.set_ylabel("[m/s]")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(times, av[:, 0], label="x")
        ax2.plot(times, av[:, 1], label="y")
        ax2.plot(times, av[:, 2], label="z")
        ax2.set_title("Angular Velocity (av)")
        ax2.set_ylabel("[rad/s]")
        ax2.set_xlabel("Time [s]")
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "velocity.png"))
        plt.close()

        # 2. Acceleration Plot (la, aa)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        ax1.plot(times, la[:, 0], label="x")
        ax1.plot(times, la[:, 1], label="y")
        ax1.plot(times, la[:, 2], label="z")
        ax1.set_title("Linear Acceleration (la)")
        ax1.set_ylabel("[m/s^2]")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(times, aa[:, 0], label="x")
        ax2.plot(times, aa[:, 1], label="y")
        ax2.plot(times, aa[:, 2], label="z")
        ax2.set_title("Angular Acceleration (aa)")
        ax2.set_ylabel("[rad/s^2]")
        ax2.set_xlabel("Time [s]")
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "acceleration.png"))
        plt.close()

        # 3. Wrench Plot (Force, Torque)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        # Force: indices 0, 1, 2
        ax1.plot(times, wrench[:, 0], label="Fx")
        ax1.plot(times, wrench[:, 1], label="Fy")
        ax1.plot(times, wrench[:, 2], label="Fz")
        ax1.set_title("Force")
        ax1.set_ylabel("[N]")
        ax1.legend()
        ax1.grid(True)

        # Torque: indices 3, 4, 5
        ax2.plot(times, wrench[:, 3], label="Tx")
        ax2.plot(times, wrench[:, 4], label="Ty")
        ax2.plot(times, wrench[:, 5], label="Tz")
        ax2.set_title("Torque")
        ax2.set_ylabel("[Nm]")
        ax2.set_xlabel("Time [s]")
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "wrench.png"))
        plt.close()

        print("Plots saved.")


if __name__ == "__main__":
    node = BatchIdentifierNode()
    # Run CLI in main thread (input() blocks)
    # ROS callbacks run in background threads automatically?
    # No, rospy constructs need spin().
    # But if we use input(), it blocks the main thread.
    # rospy.spin() blocks main thread.
    # We need a separate thread for the ROS spinning or the CLI.
    # Typically rospy.spin() is just a sleep loop.
    # Start a thread for the CLI or let ROS spin in background?
    # rospy doesn't strictly need spin() if we have our own loop.
    # But message_filters might rely on callbacks.
    # Standard pattern: Main thread does input(), background thread does processing? No, callbacks happen on data reception.
    # rospy callbacks are invoked from a separate thread if initialized?
    # Actually in Python rospy, callbacks are invoked in the main thread if we call rospy.spin(), OR we can manage it.
    # Wait, rospy.spin() is basically while not shutdown: sleep.
    # Subscriptions in rospy are handled by background threads created by init_node/Subscriber.
    # So we can just run our CLI loop in main thread and NOT call rospy.spin().

    try:
        node.run_cli()
    except rospy.ROSInterruptException:
        pass
