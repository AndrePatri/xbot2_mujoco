import time
import numpy as np
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
from xbot2_mujoco.PyXbotMjSim import XBotMjSim
from xbot2_mujoco.PyXbotMjSim import LoadingUtils


class Ros1ClockPublisher:
    def __init__(self):
        try:
            import rospy
            from rosgraph_msgs.msg import Clock
        except ImportError as exc:
            raise RuntimeError(
                "ROS 1 clock publishing requested, but rospy or rosgraph_msgs "
                "could not be imported. Source ROS 1 or use --ros-version ros2."
            ) from exc

        self._rospy = rospy
        self._clock_msg_type = Clock

        rospy.set_param('/use_sim_time', True)
        rospy.init_node('sim_clock_publisher', anonymous=True)
        self._publisher = rospy.Publisher('/clock', Clock, queue_size=10)

    def publish(self, sim_time):
        clock_msg = self._clock_msg_type(clock=self._rospy.Time.from_sec(sim_time))
        self._publisher.publish(clock_msg)

    def close(self):
        pass


class Ros2ClockPublisher:
    def __init__(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rosgraph_msgs.msg import Clock
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 clock publishing requested, but rclpy or rosgraph_msgs "
                "could not be imported. Source ROS 2 or use --ros-version ros1."
            ) from exc

        self._rclpy = rclpy
        self._clock_msg_type = Clock
        self._context = Context()
        rclpy.init(args=None, context=self._context)
        self._node = rclpy.create_node('sim_clock_publisher', context=self._context)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        if not self._node.has_parameter('use_sim_time'):
            self._node.declare_parameter('use_sim_time', True)
        self._publisher = self._node.create_publisher(Clock, '/clock', 10)

    def publish(self, sim_time):
        sec = int(sim_time)
        nanosec = int(round((sim_time - sec) * 1e9))
        if nanosec >= 1000000000:
            sec += 1
            nanosec -= 1000000000

        clock_msg = self._clock_msg_type()
        clock_msg.clock.sec = sec
        clock_msg.clock.nanosec = nanosec
        self._publisher.publish(clock_msg)
        self._executor.spin_once(timeout_sec=0.0)

    def close(self):
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        self._executor.shutdown()
        self._rclpy.shutdown(context=self._context)


def make_clock_publisher(ros_version):
    if ros_version in ("1", 1):
        ros_version = "ros1"
    elif ros_version in ("2", 2):
        ros_version = "ros2"

    if ros_version == "ros1":
        return Ros1ClockPublisher()
    if ros_version == "ros2":
        return Ros2ClockPublisher()

    raise ValueError(f"Unsupported ROS version '{ros_version}'")


def require_existing_file(path, label):
    path = Path(path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"{label} file does not exist: {path}")
    if path.stat().st_size == 0:
        raise RuntimeError(f"{label} file is empty: {path}")
    return str(path)


def require_xml_root(path, expected_root, label):
    path = require_existing_file(path, label)
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise RuntimeError(f"{label} file is not valid XML: {path}: {exc}") from exc

    if root.tag != expected_root:
        raise RuntimeError(
            f"{label} file has root <{root.tag}>, expected <{expected_root}>: {path}"
        )

    return path


class SimulatorLauncher:
    def __init__(self, args):
        self.args = args
        self.clock_publisher = None

        # Initialize the LoadingUtils instance
        self.loader = LoadingUtils("XMjEnvPy")
        files_dir = self.args.files_dir or "/root/ibrido_ws/src/xbot2_mujoco/tests/files"

        base_link = self.args.blink_name
        urdf_path = require_xml_root(
            self.args.urdf_path or f"{files_dir}/centauro/centauro.urdf",
            "robot",
            "URDF",
        )
        simopt_path = require_xml_root(
            self.args.simopt_path or f"{files_dir}/centauro/sim_opt.xml",
            "mujoco",
            "simulation options",
        )
        world_path = require_xml_root(
            self.args.world_path or f"{files_dir}/centauro/world.xml",
            "mujoco",
            "world",
        )
        sites_path = require_xml_root(
            self.args.sites_path or f"{files_dir}/centauro/sites.xml",
            "sites",
            "sites",
        )
        xbot_config_path = require_existing_file(
            self.args.xbot_config_path or f"{files_dir}/centauro/xbot2_basic.yaml",
            "XBot2 config",
        )

        # default to use centauro
        self.loader.set_urdf_path(urdf_path)
        self.loader.set_simopt_path(simopt_path)
        self.loader.set_world_path(world_path)
        self.loader.set_sites_path(sites_path)
        self.loader.set_xbot_config_path(xbot_config_path)
        self.loader.generate()

        mj_xml_path = require_xml_root(self.loader.xml_path(), "mujoco", "generated MuJoCo model")

        # Initialize the XBotMjSim environment
        self.sim = XBotMjSim(
            model_fname=mj_xml_path,
            xbot2_config_path=xbot_config_path,
            headless=self.args.headless,
            manual_stepping=not self.args.no_manual_stepping,
            init_steps=100,
            timeout=1000,
            base_link_name=base_link,
            match_rt_factor=not self.args.fullspeed,
            rt_factor_trgt=self.args.rt_factor,
            render_to_file = self.args.render_to_file,
            custom_camera_name = "custom_camera",
            render_base_path = "/tmp",
            render_fps = 60.0
        )

        if self.args.pub_rostime:
            self.clock_publisher = make_clock_publisher(self.args.ros_version)

    def quaternion_from_rotation_z(self, theta_degrees):
        # Convert theta from degrees to radians
        theta_radians = np.deg2rad(theta_degrees)

        # Calculate the quaternion components for z-axis rotation
        w = np.cos(theta_radians / 2.0)
        x = 0.0
        y = 0.0
        z = np.sin(theta_radians / 2.0)
        return [w, x, y, z]
    
    def run(self):
        np.set_printoptions(precision=2, 
                            linewidth=200)

        start_time = time.time()  # Track total time
        stepping_time = 0.0       # Track only time spent stepping
        db_stepfreq = 5000  # Frequency to print and update RT factor
        initial_step_counter = self.sim.step_counter
        
        ros_clock_freq=1

        jnt_names = self.sim.jnt_names()
        print("\nControllable joint names: ->\n")
        print(", ".join(jnt_names))

        # randomize initial pos and orientation
        import random
        pi = np.zeros((3))
        qi = np.zeros((4))
        qi[0] = 1  # Quaternion identity
        pi[2] = self.sim.get_pi()[2]  # Initial z position
        pi[0] += random.uniform(-1.0, 1.0)
        pi[1] += random.uniform(-1.0, 1.0)
        random_theta = random.uniform(-180.0, 180.0)
        
        if self.args.no_manual_stepping: # wait for sim to be ready
            while not self.sim.is_running():
                print("Waiting for sim to be ready ...")
                time.sleep(0.5)

        qi[:] = self.quaternion_from_rotation_z(random_theta)
        self.sim.set_pi(pi)
        self.sim.set_qi(qi)
        self.sim.reset()
        
        while self.sim.is_running():
            step_start = time.time()  # Start timing the step
            step_ok=self.sim.step()
            if not step_ok:
                break
            step_end = time.time()  # End timing the step
            stepping_time += (step_end - step_start)  # Accumulate stepping time

            # Publish simulation time to /clock if enabled
            if self.clock_publisher and (self.sim.step_counter % ros_clock_freq == 0):
                simtime_elapsed = self.sim.physics_dt * self.sim.step_counter
                self.clock_publisher.publish(simtime_elapsed)

            # Update and print RT factor every db_stepfreq steps
            if self.sim.step_counter % db_stepfreq == 0:
                elapsed_time = time.time() - start_time  # Total elapsed wall time so far
                simtime_elapsed = self.sim.physics_dt * self.sim.step_counter
                rt_factor = simtime_elapsed / elapsed_time if elapsed_time > 0 else float('inf')
                print(f"RT Factor at step {self.sim.step_counter}: {rt_factor:.2f}, \
                      Simulated Time: {simtime_elapsed:.6f}s, Elapsed Time: {elapsed_time:.6f}s")
                print("Robot position")
                print(self.sim.p)
                print("Robot orientation")
                print(self.sim.q)

        # Final stats after simulation ends
        total_elapsed_time = time.time() - start_time  # Total wall time (including everything)
        total_steps_done = self.sim.step_counter - initial_step_counter
        simtime_elapsed = self.sim.physics_dt * total_steps_done
        stepping_rt_factor = simtime_elapsed / stepping_time if stepping_time > 0 else float('inf')

        print("\nSimulation Finished:")
        print(f"Number of timesteps done: {total_steps_done}")
        print(f"Total elapsed wall time: {total_elapsed_time:.6f} seconds (including all operations).")
        print(f"Actual stepping time: {stepping_time:.6f} seconds.")
        print(f"Simulated time: {simtime_elapsed:.6f} seconds.")
        print(f"RT factor (total elapsed time): {simtime_elapsed / total_elapsed_time:.2f}")
        print(f"RT factor (actual stepping time): {stepping_rt_factor:.2f}, physics dt: {self.sim.physics_dt:.6f} seconds.")

    def close(self):
        try:
            self.sim.close()
        finally:
            if self.clock_publisher:
                self.clock_publisher.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Launch XBotMjSim simulation.')

    # File paths
    parser.add_argument('--files_dir', type=str, help='Base directory for simulation files')
    parser.add_argument('--urdf_path', type=str, help='Full path to the URDF file to be compiled and loaded.')
    parser.add_argument('--simopt_path', type=str, help='Path to the simulation options file ')
    parser.add_argument('--world_path', type=str, help='Path to the world file ')
    parser.add_argument('--sites_path', type=str, help='Path to the sites file ')
    parser.add_argument('--xbot_config_path', type=str, help='Path to the XBot2 configuration file.')

    # Simulation parameters
    parser.add_argument('--no_manual_stepping', action='store_true', help='disable manual stepping')
    parser.add_argument('--headless', action='store_true', help='Run the simulation in headless mode.')
    parser.add_argument('--pub_rostime', action='store_true', help='Publish simulation time to the /clock topic.')
    parser.add_argument('--ros-version', '--ros_version', dest='ros_version',
        choices=('ros2', 'ros1', '2', '1'), default='ros2',
        help='ROS backend used when --pub_rostime is enabled.')
    parser.add_argument('--blink_name', type=str, default="base_link", 
        help='root link name (will be used for getting measurements and teleportation)')
    parser.add_argument('--fullspeed', action='store_true', help='Do NOT try to match desired rt factor')
    parser.add_argument('--rt_factor', type=float, help='target rt factor', default=1.0)
    parser.add_argument('--render_to_file', action='store_true', help='')

    args = parser.parse_args()

    simulator = SimulatorLauncher(args)
    try:
        simulator.run()
    finally:
        simulator.close()
