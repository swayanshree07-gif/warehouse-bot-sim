"""OmniGraph action graphs that connect the sim to ROS 2.

Import after SimulationApp exists and after enable_ros_extensions().
"""
import omni.graph.core as og
import omni.kit.app

import robot_params as P

EXTENSIONS = (
    "isaacsim.ros2.bridge",
    "isaacsim.robot.wheeled_robots.nodes",
    "isaacsim.core.nodes",
    "isaacsim.sensors.experimental.physics",
    "isaacsim.sensors.physics.nodes",
)


def _t(ns, name):
    """Topic name inside namespace ns (Day 19 fleet); '' keeps the old topic."""
    return f"{ns}/{name}" if ns else name


def enable_ros_extensions(app):
    mgr = omni.kit.app.get_app().get_extension_manager()
    for ext in EXTENSIONS:
        mgr.set_extension_enabled_immediate(ext, True)
    app.update()
    missing = [e for e in EXTENSIONS if not mgr.is_extension_enabled(e)]
    if missing:
        raise RuntimeError(f"extensions not enabled: {missing}")


def build_clock_graph(path="/World/ROS/ClockGraph"):
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )


def build_cmd_vel_graph(robot_path="/World/Robot", topic="cmd_vel", path="/World/ROS/CmdVelGraph"):
    """/cmd_vel (Twist) -> DifferentialController -> wheel velocity targets.

    Note: the Twist subscriber keeps the last command; senders must publish a
    zero twist to stop (Nav2 and teleop do). A command watchdog may come later.
    """
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLin", "omni.graph.nodes.BreakVector3"),
                ("BreakAng", "omni.graph.nodes.BreakVector3"),
                ("DiffCtrl", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArtCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "DiffCtrl.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArtCtrl.inputs:execIn"),
                ("OnPlaybackTick.outputs:deltaSeconds", "DiffCtrl.inputs:dt"),
                ("SubTwist.outputs:linearVelocity", "BreakLin.inputs:tuple"),
                ("SubTwist.outputs:angularVelocity", "BreakAng.inputs:tuple"),
                ("BreakLin.outputs:x", "DiffCtrl.inputs:linearVelocity"),
                ("BreakAng.outputs:z", "DiffCtrl.inputs:angularVelocity"),
                ("DiffCtrl.outputs:velocityCommand", "ArtCtrl.inputs:velocityCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubTwist.inputs:topicName", topic),
                ("DiffCtrl.inputs:wheelRadius", P.WHEEL_RADIUS),
                ("DiffCtrl.inputs:wheelDistance", P.WHEEL_SEPARATION),
                ("DiffCtrl.inputs:maxLinearSpeed", P.MAX_LINEAR_SPEED),
                ("DiffCtrl.inputs:maxAngularSpeed", P.MAX_ANGULAR_SPEED),
                ("ArtCtrl.inputs:robotPath", robot_path),
                ("ArtCtrl.inputs:jointNames", ["left_wheel_joint", "right_wheel_joint"]),
            ],
        },
    )


def build_odom_graph(robot_path="/World/Robot", chassis="chassis", path="/World/ROS/OdomGraph",
                     odom_frame="odom", base_frame="base_link", ns=""):
    """/odom (nav_msgs/Odometry), TF odom->base_link and /joint_states.

    Odometry comes from IsaacComputeOdometry on the chassis rigid body (ground
    truth, relative to the pose at playback start), so base_link = chassis
    centre. Stamps use simulation time (pair with use_sim_time + /clock).
    """
    chassis_path = f"{robot_path}/{chassis}"
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PubTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("PubJoints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PubOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PubTf.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PubJoints.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PubTf.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PubJoints.inputs:timeStamp"),
                ("ComputeOdom.outputs:position", "PubOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PubOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
                ("ComputeOdom.outputs:position", "PubTf.inputs:translation"),
                ("ComputeOdom.outputs:orientation", "PubTf.inputs:rotation"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ComputeOdom.inputs:chassisPrim", [chassis_path]),
                ("PubOdom.inputs:topicName", _t(ns, "odom")),
                ("PubOdom.inputs:odomFrameId", odom_frame),
                ("PubOdom.inputs:chassisFrameId", base_frame),
                ("PubTf.inputs:parentFrameId", odom_frame),
                ("PubTf.inputs:childFrameId", base_frame),
                ("PubTf.inputs:topicName", "tf"),
                ("PubJoints.inputs:targetPrim", [robot_path]),
                ("PubJoints.inputs:topicName", _t(ns, "joint_states")),
            ],
        },
    )


def build_scan_graph(sensor_path, path="/World/ROS/ScanGraph", frame="laser", base_frame="base_link",
                     physics_hz=60.0, ns=""):
    """/scan (sensor_msgs/LaserScan) from the raycast lidar + static TF
    base_link -> laser. Published every N ticks for ~P.LASER_RATE_HZ."""
    import usdrt.Sdf

    n = len(P.laser_angles_deg())
    step = max(1, int(round(physics_hz / P.LASER_RATE_HZ)))
    og.Controller.edit(
        {"graph_path": path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Gate", "isaacsim.core.nodes.IsaacSimulationGate"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadRaycast", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
                ("PubScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                ("PubLaserTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "Gate.inputs:execIn"),
                ("Gate.outputs:execOut", "ReadRaycast.inputs:execIn"),
                ("ReadRaycast.outputs:execOut", "PubScan.inputs:execIn"),
                ("ReadRaycast.outputs:depths", "PubScan.inputs:linearDepthData"),
                ("ReadSimTime.outputs:simulationTime", "PubScan.inputs:timeStamp"),
                ("Gate.outputs:execOut", "PubLaserTf.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PubLaserTf.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Gate.inputs:step", step),
                ("ReadRaycast.inputs:raycastSensorPrim", [usdrt.Sdf.Path(sensor_path)]),
                ("PubScan.inputs:topicName", _t(ns, "scan")),
                ("PubScan.inputs:frameId", frame),
                ("PubScan.inputs:horizontalFov", 360.0),
                ("PubScan.inputs:horizontalResolution", P.LASER_RES_DEG),
                ("PubScan.inputs:numCols", n),
                ("PubScan.inputs:numRows", 1),
                ("PubScan.inputs:depthRange", list(P.LASER_RANGE)),
                ("PubScan.inputs:rotationRate", 0.0),
                ("PubScan.inputs:azimuthRange", [-180.0, 180.0]),
                ("PubLaserTf.inputs:parentFrameId", base_frame),
                ("PubLaserTf.inputs:childFrameId", frame),
                ("PubLaserTf.inputs:translation", list(P.LASER_POS)),
                ("PubLaserTf.inputs:staticPublisher", True),
                ("PubLaserTf.inputs:topicName", "tf_static"),
            ],
        },
    )
