import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import subprocess
import time
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

        # ROS2 Publisher
        self.odom_pub = []
        self.pose_pub = []

        # ROS2 Subscriber
        self.cmd_vel_sub = []
        self.odom_pub.append(
            self.create_publisher(Odometry, "unitree_go2/odom", 10))
        self.pose_pub.append(
            self.create_publisher(PoseStamped, "unitree_go2/pose", 10))
        self.cmd_vel_sub.append(
            self.create_subscription(Twist, "unitree_go2/cmd_vel",
            lambda msg: self.cmd_vel_callback(msg, 0), 10)
        )

        # use wall time for odom pub
        self.odom_pose_freq = 50.0
        self.odom_pose_pub_time = time.time()
        self.create_static_transform()
        self.create_camera_publisher()


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

    def publish_odom(self, base_pos, base_rot, base_lin_vel_b, base_ang_vel_b, env_idx):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = base_pos[0].item()
        odom_msg.pose.pose.position.y = base_pos[1].item()
        odom_msg.pose.pose.position.z = base_pos[2].item()
        odom_msg.pose.pose.orientation.x = base_rot[1].item()
        odom_msg.pose.pose.orientation.y = base_rot[2].item()
        odom_msg.pose.pose.orientation.z = base_rot[3].item()
        odom_msg.pose.pose.orientation.w = base_rot[0].item()
        odom_msg.twist.twist.linear.x = base_lin_vel_b[0].item()
        odom_msg.twist.twist.linear.y = base_lin_vel_b[1].item()
        odom_msg.twist.twist.linear.z = base_lin_vel_b[2].item()
        odom_msg.twist.twist.angular.x = base_ang_vel_b[0].item()
        odom_msg.twist.twist.angular.y = base_ang_vel_b[1].item()
        odom_msg.twist.twist.angular.z = base_ang_vel_b[2].item()
        self.odom_pub[env_idx].publish(odom_msg)

        # transform
        map_base_trans = TransformStamped()
        map_base_trans.header.stamp = self.get_clock().now().to_msg()
        map_base_trans.header.frame_id = "map"
        map_base_trans.child_frame_id = "unitree_go2/base_link"
        map_base_trans.transform.translation.x = base_pos[0].item()
        map_base_trans.transform.translation.y = base_pos[1].item()
        map_base_trans.transform.translation.z = base_pos[2].item()
        map_base_trans.transform.rotation.x = base_rot[1].item()
        map_base_trans.transform.rotation.y = base_rot[2].item()
        map_base_trans.transform.rotation.z = base_rot[3].item()
        map_base_trans.transform.rotation.w = base_rot[0].item()
        self.broadcaster.sendTransform(map_base_trans)

    def publish_pose(self, base_pos, base_rot, env_idx):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = base_pos[0].item()
        pose_msg.pose.position.y = base_pos[1].item()
        pose_msg.pose.position.z = base_pos[2].item()
        pose_msg.pose.orientation.x = base_rot[1].item()
        pose_msg.pose.orientation.y = base_rot[2].item()
        pose_msg.pose.orientation.z = base_rot[3].item()
        pose_msg.pose.orientation.w = base_rot[0].item()
        self.pose_pub[env_idx].publish(pose_msg)

    def pub_ros2_data(self):
        pub_odom_pose = False
        dt_odom_pose = time.time() - self.odom_pose_pub_time
        if (dt_odom_pose >= 1./self.odom_pose_freq):
            pub_odom_pose = True

        if (pub_odom_pose):
            self.odom_pose_pub_time = time.time()
            robot_data = self.env.unwrapped.scene["unitree_go2"].data
            for i in range(self.num_envs):
                self.publish_odom(robot_data.root_state_w[i, :3],
                                robot_data.root_state_w[i, 3:7],
                                robot_data.root_lin_vel_b[i],
                                robot_data.root_ang_vel_b[i],
                                i)
                self.publish_pose(robot_data.root_state_w[i, :3],
                                robot_data.root_state_w[i, 3:7], i)

    def create_camera_publisher(self):
        if (self.cfg.sensor.enable_camera):
            if (self.cfg.sensor.color_image):
                self.pub_color_image()
            if (self.cfg.sensor.depth_image):
                self.pub_depth_image()

    def pub_color_image(self):
        for i in range(self.num_envs):
            # The following code will link the camera's render product and publish the data to the specified topic name.
            render_product = self.cameras[i]._render_product_path
            step_size = 1
            topic_name = "unitree_go2/front_cam/color_image"
            frame_id = "unitree_go2/front_cam"
            node_namespace = ""
            queue_size = 1

            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

            writer = rep.writers.get(rv + "ROS2PublishImage")
            writer.initialize(
                frameId=frame_id,
                nodeNamespace=node_namespace,
                queueSize=queue_size,
                topicName=topic_name,
            )
            writer.attach([render_product])

            # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
            gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                rv + "IsaacSimulationGate", render_product
            )
            og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    def pub_depth_image(self):
        for i in range(self.num_envs):
            # The following code will link the camera's render product and publish the data to the specified topic name.
            render_product = self.cameras[i]._render_product_path
            step_size = 1
            topic_name = "unitree_go2/front_cam/depth_image"
            frame_id = "unitree_go2/front_cam"
            node_namespace = ""
            queue_size = 1

            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                                    sd.SensorType.DistanceToImagePlane.name
                                )
            writer = rep.writers.get(rv + "ROS2PublishImage")
            writer.initialize(
                frameId=frame_id,
                nodeNamespace=node_namespace,
                queueSize=queue_size,
                topicName=topic_name
            )
            writer.attach([render_product])

            # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
            gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                rv + "IsaacSimulationGate", render_product
            )
            og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    def cmd_vel_callback(self, msg, env_idx):
        go2_ctrl.base_vel_cmd_input[env_idx][0] = msg.linear.x
        go2_ctrl.base_vel_cmd_input[env_idx][1] = msg.linear.y
        go2_ctrl.base_vel_cmd_input[env_idx][2] = msg.angular.z
