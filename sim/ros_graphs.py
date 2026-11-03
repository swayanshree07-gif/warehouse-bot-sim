"""OmniGraph action graphs that connect the sim to ROS 2.

Import after SimulationApp exists and after enable_ros_extensions().
"""
import omni.graph.core as og
import omni.kit.app

import robot_params as P

EXTENSIONS = ("isaacsim.ros2.bridge", "isaacsim.robot.wheeled_robots.nodes", "isaacsim.core.nodes")


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
