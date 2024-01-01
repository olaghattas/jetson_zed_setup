import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import os
import sys
import re


class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.initial_command_process = None
        self.flag = False
        self.transform = None

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
        self.transform = t
        self.flag = True
        self.tf_static_broadcaster.sendTransform(t)

    def file_exists(self, file_path):
        return os.path.exists(file_path)


def main():
    rclpy.init()
    node = TFListener()
    while True:
        try:
            if node.flag:
                print("*** In if node.flag ***")
                node.tf_static_broadcaster.sendTransform(node.transform)

            initial_command = sys.argv[1]  # "ros2 launch apriltag_ros tag_zed.launch.py camera_name:=/zed_doorway/zed_node_doorway/left image_topic:=image_rect_color apriltag_detections:=/apriltag_detections_zed_doorway"
            source_frame = sys.argv[2]  # "zed_kitchen_left_camera_frame"
            target_frame = sys.argv[3]  # "tag_" + os.environ.get("target_id") + "_zed"
            file_path = sys.argv[4]  # 'your_file.txt'
            # "zed_kitchen_left_camera_frame"  "tag_7_zed" "your_file.txt"

            print("Source frame: ", source_frame)
            print("Target Frame: ", target_frame)
            print("File path: ", file_path)
            # Wait for the TF transform between source_frame and target_frame
            if node.file_exists(file_path):
                print("&&&& file read %$$$$")
                with open(file_path, 'r') as file:
                    x, y, z, qx, qy, qz, qw = map(float, file.readline().split())

                t = TransformStamped()

                t.header.stamp = node.get_clock().now().to_msg()
                t.header.frame_id = source_frame
                t.child_frame_id = target_frame

                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = z
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw

                node.tf_static_broadcaster.sendTransform(t)
                node.transform = t
                node.flag = True
                print("Node: ", node)

            # Save the tf_transform, source_frame, and target_frame in a file
            else:
                # Run the initial command
                node.run_initial_command(initial_command)
                print("&&&& file doesnt exists %$$$$")
                tf_transform = node.wait_for_tf_transform(source_frame, target_frame)
                if tf_transform is not None:
                    node.get_logger().info(f"TF transform between {source_frame} and {target_frame} found.")
                    print("&&&& file saved %$$$$")
                    print("tf_transform dfdfgd ",tf_transform)
                    with open(file_path, 'w') as file:
                        file.write(
                            f"{tf_transform.transform.translation.x} {tf_transform.transform.translation.y} {tf_transform.transform.translation.z} "
                            f"{tf_transform.transform.rotation.x} {tf_transform.transform.rotation.y} {tf_transform.transform.rotation.z} {tf_transform.transform.rotation.w}")
                    print("&&&& file saved %$$$$")
                    node.make_transform(source_frame, target_frame, tf_transform.transform)
                    # Stop the initial command
                    node.stop_initial_command()
                else:
                    node.get_logger().info(f"Timeout waiting for TF transform between {source_frame} and {target_frame}.")
        except Exception as e:
            print("ERror in try", e)
        time.sleep(10)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
