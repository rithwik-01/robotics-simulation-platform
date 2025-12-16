import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import omni
import omni.graph.core as og
import subprocess
import go2.go2_ctrl as go2_ctrl

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)


class RobotDataManager(Node):
    def __init__(self, env, lidar_annotators, cameras, cfg):
        super().__init__("robot_data_manager")
        self.cfg = cfg
        self.create_ros_time_graph()
        sim_time_set = False
        while (rclpy.ok() and sim_time_set==False):
            sim_time_set = self.use_sim_time()

        self.env = env
        self.num_envs = env.unwrapped.scene.num_envs
        self.lidar_annotators = lidar_annotators
        self.cameras = cameras

        # ROS2 Broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # ROS2 Subscriber
        self.cmd_vel_sub = []
        self.cmd_vel_sub.append(
            self.create_subscription(Twist, "unitree_go2/cmd_vel",
            lambda msg: self.cmd_vel_callback(msg, 0), 10)
        )

        self.create_static_transform()


    def create_ros_time_graph(self):
        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),

                    # Connecting simulationTime data of ReadSimTime to the clock publisher node
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Assigning topic name to clock publisher
                    ("PublishClock.inputs:topicName", "/clock"),
                ],
            },
        )

    def use_sim_time(self):
        # Define the command as a list
        command = ["ros2", "param", "set", "/robot_data_manager", "use_sim_time", "true"]

        # Run the command in a non-blocking way
        subprocess.Popen(command)
        return True

    def create_static_transform(self):
        for i in range(self.num_envs):
            # LiDAR
            # Create and publish the transform
            lidar_broadcaster = StaticTransformBroadcaster(self)
            base_lidar_transform = TransformStamped()
            base_lidar_transform.header.stamp = self.get_clock().now().to_msg()
            base_lidar_transform.header.frame_id = "unitree_go2/base_link"
            base_lidar_transform.child_frame_id = "unitree_go2/lidar_frame"

            # Translation
            base_lidar_transform.transform.translation.x = 0.2
            base_lidar_transform.transform.translation.y = 0.0
            base_lidar_transform.transform.translation.z = 0.2

            # Rotation
            base_lidar_transform.transform.rotation.x = 0.0
            base_lidar_transform.transform.rotation.y = 0.0
            base_lidar_transform.transform.rotation.z = 0.0
            base_lidar_transform.transform.rotation.w = 1.0

            # Publish the transform
            lidar_broadcaster.sendTransform(base_lidar_transform)

            # -------------------------------------------------------------
            # Camera
            # Create and publish the transform
            camera_broadcaster = StaticTransformBroadcaster(self)
            base_cam_transform = TransformStamped()
            base_cam_transform.header.frame_id = "unitree_go2/base_link"
            base_cam_transform.child_frame_id = "unitree_go2/front_cam"

            # Translation
            base_cam_transform.transform.translation.x = 0.4
            base_cam_transform.transform.translation.y = 0.0
            base_cam_transform.transform.translation.z = 0.2

            # Rotation
            base_cam_transform.transform.rotation.x = -0.5
            base_cam_transform.transform.rotation.y = 0.5
            base_cam_transform.transform.rotation.z = -0.5
            base_cam_transform.transform.rotation.w = 0.5

            # Publish the transform
            camera_broadcaster.sendTransform(base_cam_transform)

    def cmd_vel_callback(self, msg, env_idx):
        go2_ctrl.base_vel_cmd_input[env_idx][0] = msg.linear.x
        go2_ctrl.base_vel_cmd_input[env_idx][1] = msg.linear.y
        go2_ctrl.base_vel_cmd_input[env_idx][2] = msg.angular.z
