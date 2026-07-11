from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    EmitEvent,
    RegisterEventHandler,
)
from launch.substitutions import TextSubstitution
from launch_ros.actions import LifecycleNode
from launch.event_handlers import OnProcessStart
from launch.event_handlers.on_shutdown import OnShutdown
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def launch_setup(context):
    description = []
    serial_driver_onde = LifecycleNode(
        package='serial_driver',
        executable='serial_bridge',
        name='serial_bridge_node',
        namespace=TextSubstitution(text=''),
        parameters=[{
                    'device_name': '/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0',  # The "blue" serial cable
                    'baud_rate': 115200,
                    'flow_control': 'none',
                    'parity': 'none',
                    'stop_bits': "1"}],
        remappings=[("serial_read", "rscp/serial/rx"),
                    ("serial_write", "rscp/serial/tx")],
        output='screen',
    )
    configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=serial_driver_onde,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(serial_driver_onde),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        )
    )

    activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=serial_driver_onde,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(serial_driver_onde),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        )
    )

    shutdown_event_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(serial_driver_onde),
                        transition_id=Transition.TRANSITION_ACTIVE_SHUTDOWN,
                    )
                )
            ]
        )
    )
    description += [
        serial_driver_onde,
        configure_event_handler,
        activate_event_handler,
        shutdown_event_handler,
    ]


    return description


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
