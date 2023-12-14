import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import os


class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.initial_command_process = None

    def run_initial_command(self, command):
        self.initial_command_process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
        self.get_logger().info(f"Running initial command: {command}")

    def wait_for_tf_transform(self, source_frame, target_frame, timeout_sec=100):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(),
                                                            timeout=rclpy.duration.Duration(seconds=5.0))
                return transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # TF transform not available yet, continue waiting
                rclpy.spin_once(self, timeout_sec=1)

        return None

    def stop_initial_command(self):
        if self.initial_command_process:
            command = 'pkill -f "component_container"'
            subprocess.run(command, shell=True)
            self.get_logger().info(f"Running alternative command: {command}")


    def run_alternative_command(self, command):
        subprocess.run(command, shell=True)
        self.get_logger().info(f"Running alternative command: {command}")

    def make_transform(self, source_frame, target_frame, transform):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = source_frame
        t.child_frame_id = target_frame

        t.transform.translation = transform.translation
        t.transform.rotation = transform.rotation

        self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TFListener()

    try:
        initial_command = "ros2 launch apriltag_ros tag_zed.launch.py"
        alternative_command = "ros2 run particle_filter_mesh particle_filter_node"
        source_frame = "zed_kitchen_left_camera_frame"
        target_frame = "tag_" + os.environ.get("target_id") + "_zed"

        # Run the initial command
        node.run_initial_command(initial_command)
        # Wait for the TF transform between source_frame and target_frame
        tf_transform = node.wait_for_tf_transform(source_frame, target_frame)
        if tf_transform is not None:
            node.get_logger().info(f"TF transform between {source_frame} and {target_frame} found.")
            node.make_transform(source_frame, target_frame, tf_transform.transform)
            # Stop the initial command
            node.stop_initial_command()

            # Run the alternative command
            node.run_alternative_command(alternative_command)
        else:
            node.get_logger().info(f"Timeout waiting for TF transform between {source_frame} and {target_frame}.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
