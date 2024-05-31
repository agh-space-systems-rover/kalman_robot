from __future__ import annotations

import traceback
import time
from threading import Lock, Condition, Event
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.action.client import GoalStatus, ClientGoalHandle
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from unique_identifier_msgs.msg import UUID

from kalman_master.ros_link_serialization import serialize_message, deserialize_message

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from kalman_master.ros_link_node import RosLink
    from rclpy.action.server import ServerGoalHandle


def goal_id_to_hex_str(goal_id: UUID) -> str:
    return "".join([f"{byte:02x}" for byte in goal_id.uuid])


class IncomingGoalState:
    def __init__(self, client_handle: ClientGoalHandle):
        self.client_handle = client_handle
        self.cancelled: bool = False


class OutgoingGoalState:
    def __init__(self, server_handle: ServerGoalHandle):
        self.server_handle = server_handle
        self.status: int | None = None
        self.result: Any = None
        self.last_feedback: Any = None
        # When a cancel ACK is received, this is set to True if the cancel was successful as indicated by the ACK payload.
        self.cancelled_state_from_ack: bool = False


class RosLinkActions:
    def __init__(self, node: RosLink):
        self.node = node

        # Create action clients for actions provided by this side.
        self.action_clients: dict[str, ActionClient] = {}
        for action_config in self.node.get_this_side_config()["actions"]:
            self.action_clients[action_config["name"]] = ActionClient(
                self.node,
                self.node.imported_interfaces[action_config["type"]],
                action_config["name"],
            )

        # Create action servers for actions provided by the opposite side.
        self.action_servers: dict[str, ActionServer] = {}
        for action_config in self.node.get_opposite_side_config()["actions"]:
            self.action_servers[action_config["name"]] = ActionServer(
                self.node,
                self.node.imported_interfaces[action_config["type"]],
                action_config["name"]
                + ("/loopback" if self.node.loopback_mangling else ""),
                lambda goal_handle, action_config=action_config: self.execute_callback(
                    goal_handle, action_config
                ),
                callback_group=self.node.callback_group,
                cancel_callback=lambda goal_handle, action_config=action_config: self.cancel_callback(
                    goal_handle, action_config
                ),
            )

        # Generates septuple ID sequences for unique action calls. IDs are generated in execute_callback.
        self.outgoing_action_req_id_counter = 0
        self.outgoing_action_req_id_counter_lock = Lock()
        # Used to ignore duplicate request/cancel/response frames and to check whether an ACK was received. Also used to ignore response ACKs for calls that are not pending anymore. A call to action on this side is pending if its request ID is in this set.
        self.received_action_sequence_ids: set[int] = set()
        self.received_action_sequence_ids_condition = Condition()
        # Used to ignore request ACKs for requests that are not pending. A call to an action on the opposite side is pending if its request ID is in this dictionary. Additionally used to store responses from actions hosted on the opposite side. The responses are received by master_callback, but they need to be passed to another thread that is waiting in execute_callback.
        self.outgoing_action_goal_states: dict[int, OutgoingGoalState] = {}
        self.outgoing_action_goal_states_condition = Condition()
        # This is used to cancel the action when a cancel frame is received.
        self.incoming_action_goal_states: dict[int, IncomingGoalState] = {}
        self.incoming_action_goal_states_lock = Lock()
        # Map of publication times of feedback messages for each action call. Used to rate limit the feedback topic.
        self.last_feedback_times: dict[str, float] = {}

    # Get the configuration for an action.
    def get_action_config(self, name: str) -> dict:
        for side_config in [
            self.node.get_this_side_config(),
            self.node.get_opposite_side_config(),
        ]:
            for action_config in side_config["actions"]:
                if action_config["name"] == name:
                    return action_config
        raise ValueError(f"No action configuration found for action {name}.")

    # Called by master_callback when an action frame is received.
    def handle_action_frame(self, action_name: str, data: list) -> None:
        # Skip if the frame does not contain sequence ID.
        if len(data) < 1:
            self.node.get_logger().error(
                f"Master delivered an action frame without sequence ID for action {action_name}."
            )
            return

        seq_id = data[0]
        data = data[1:]  # Skip sequence ID.

        # If this is a request frame...
        if seq_id % 7 == 0:
            self.handle_action_request_frame(action_name, seq_id, data)
        # If this is a request ACK frame...
        elif seq_id % 7 == 1:
            self.handle_action_request_ack_frame(action_name, seq_id)
        # If this is a response frame...
        elif seq_id % 7 == 2:
            self.handle_action_response_frame(action_name, seq_id, data)
        # If this is a response ACK frame...
        elif seq_id % 7 == 3:
            self.handle_action_response_ack_frame(action_name, seq_id)
        # If this is a feedback frame...
        elif seq_id % 7 == 4:
            self.handle_action_feedback_frame(action_name, seq_id, data)
        # If this is a cancel frame...
        elif seq_id % 7 == 5:
            self.handle_action_cancel_frame(action_name, seq_id, data)
        # If this is a cancel ACK frame...
        elif seq_id % 7 == 6:
            self.handle_action_cancel_ack_frame(action_name, seq_id, data)

    # Called by handle_action_frame when a action request frame is received.
    def handle_action_request_frame(
        self, action_name: str, req_id: int, data: list
    ) -> None:
        if self.node.debug_info:
            self.node.get_logger().info(f"{action_name}#{req_id} request")

        action_id = self.node.ordered_names.index(action_name)

        # Send one ACK frame.
        self.node.send_to_opposite_side([action_id, req_id + 1])  # ACK is always ID + 1

        # If this frame ID was not received before...
        self.received_action_sequence_ids_condition.acquire()
        if req_id not in self.received_action_sequence_ids:
            # Clear all other IDs reserved for this side from the received set.
            # action request >
            # request ACK <
            # action response <
            # response ACK >
            # action feedback <
            # action cancel >
            # cancel ACK <
            # if req_id in self.received_action_sequence_ids: # already cleared because of the if statement
            #     self.received_action_sequence_ids.remove(req_id) # request ID
            if req_id + 3 in self.received_action_sequence_ids:
                self.received_action_sequence_ids.remove(req_id + 3)  # response ACK ID
            if req_id + 5 in self.received_action_sequence_ids:
                self.received_action_sequence_ids.remove(req_id + 5)  # cancel ID

            # Save the frame ID as received.
            self.received_action_sequence_ids.add(req_id)
            self.received_action_sequence_ids_condition.notify_all()
            self.received_action_sequence_ids_condition.release()

            # Clear the goal handle from the dictionary.
            with self.incoming_action_goal_states_lock:
                if req_id in self.incoming_action_goal_states:
                    self.incoming_action_goal_states.pop(req_id)

            # Deserialize the request.
            action_config = self.get_action_config(action_name)
            try:
                goal = deserialize_message(
                    data,
                    self.node.imported_interfaces[action_config["type"]].Goal,
                    action_config["goal_fields"],
                )
            except ValueError as e:
                self.node.get_logger().error(
                    f"Failed to deserialize a goal request to action {action_config['name']}:\n{e}"
                )
                return
            except Exception:
                self.node.get_logger().error(
                    f"Unexpected error while deserializing a goal request to action {action_config['name']}:\n{traceback.format_exc()}"
                )
                return

            if self.node.debug_info:
                self.node.get_logger().info(
                    f"Calling the action {action_config['name']} as per request ID {req_id}."
                )

            # Call the action.
            last_waiting_for_action_log_time = 0
            while not self.action_clients[action_name].server_is_ready():
                if time.time() - last_waiting_for_action_log_time > 5:
                    self.node.get_logger().info(
                        f"Waiting for action server {action_name} to be ready..."
                    )
                    last_waiting_for_action_log_time = time.time()
                time.sleep(
                    0.01
                )  # TODO: Maybe support cancelling the action while waiting for the server to be ready? Not sure if it is very useful though.

            def feedback_callback(
                feedback,
                action_id=action_id,
                req_id=req_id,
                action_config=action_config,
            ):
                if action_config["name"] in self.last_feedback_times:
                    if (
                        time.time() - self.last_feedback_times[action_config["name"]]
                        < 1 / action_config["max_feedback_rate"]
                    ):
                        return
                self.last_feedback_times[action_config["name"]] = time.time()

                feedback_data = serialize_message(
                    feedback.feedback, action_config["feedback_fields"]
                )
                self.node.send_to_opposite_side([action_id, req_id + 4] + feedback_data)

            future = self.action_clients[action_name].send_goal_async(
                goal, feedback_callback=feedback_callback
            )
            event = Event()

            def done_callback(_):
                nonlocal event
                event.set()

            future.add_done_callback(done_callback)
            event.wait()
            goal_handle: ClientGoalHandle = future.result()

            # Save the goal handle for possible cancellation requests.
            with self.incoming_action_goal_states_lock:
                self.incoming_action_goal_states[req_id] = IncomingGoalState(
                    client_handle=goal_handle
                )

            if self.node.debug_info:
                self.node.get_logger().info(
                    f"Call successful, waiting for result of {action_config['name']}#{req_id}."
                )

            # Wait for the result.
            future = goal_handle.get_result_async()
            event = Event()

            def done_callback(_):
                nonlocal event
                event.set()

            future.add_done_callback(done_callback)
            action_server_died = False
            while event.is_set() is False:
                # From time to time while waiting, check if server is still alive.
                if not self.action_clients[action_name].server_is_ready():
                    self.node.get_logger().error(
                        f"Action server {action_name} appears to have died. Aborting the action."
                    )
                    action_server_died = True
                    break
                event.wait(timeout=5)  # Wait for 5 seconds for the result.
            if not action_server_died:
                status_and_result = future.result()
                status: int = status_and_result.status
                result = status_and_result.result  # type is MyAction.Result
            else:
                status = GoalStatus.STATUS_ABORTED
                # result is not needed as it won't be serialized.

            # If goal was successful, serialize a response.
            if status == GoalStatus.STATUS_SUCCEEDED:
                # Serialize the response.
                try:
                    data = serialize_message(result, action_config["result_fields"])
                except ValueError as e:
                    self.node.get_logger().error(
                        f"Failed to serialize result of action {action_config['name']}:\n{e}"
                    )
                    return
                except Exception:
                    self.node.get_logger().error(
                        f"Unexpected error while serializing a result of action {action_config['name']}:\n{traceback.format_exc()}"
                    )
                    return
            else:
                data = []

            if self.node.debug_info:
                self.node.get_logger().info(
                    f"Action call has finished for {action_config['name']}#{req_id}. Sending response frames..."
                )

            # Keep publishing the response frame until it is ACKed.
            self.received_action_sequence_ids_condition.acquire()
            while (
                req_id + 3 not in self.received_action_sequence_ids
            ):  # Response ACK is request ID + 3
                self.received_action_sequence_ids_condition.release()
                self.node.send_to_opposite_side(
                    [action_id, req_id + 2, status] + data
                )  # Response is request ID + 2
                self.received_action_sequence_ids_condition.acquire()
                self.received_action_sequence_ids_condition.wait(
                    1 / action_config["retry_rate"]
                )
            self.received_action_sequence_ids_condition.release()

            if self.node.debug_info:
                self.node.get_logger().info(
                    f"Received response ACK for action {action_config['name']}. Call for request ID {req_id} is finalized."
                )

            # Clear request ID from received_action_sequence_ids to free it for future calls.
            self.received_action_sequence_ids_condition.acquire()
            if req_id in self.received_action_sequence_ids:
                self.received_action_sequence_ids.remove(req_id)  # request ID
                self.received_action_sequence_ids_condition.notify_all()
        try:
            self.received_action_sequence_ids_condition.release()
        except RuntimeError:
            pass

    # Called by handle_action_frame when an action request ACK frame is received.
    def handle_action_request_ack_frame(
        self, action_name: str, req_ack_id: int
    ) -> None:
        if self.node.debug_info:
            self.node.get_logger().info(f"{action_name}#{req_ack_id - 1} request ACK")

        # Ignore if no such request is pending.
        if req_ack_id - 1 not in self.outgoing_action_goal_states:  # synced by GIL
            # This might happen when extra request ACKs are sent after the call sequence was finalized.
            return
        with self.received_action_sequence_ids_condition:
            self.received_action_sequence_ids.add(req_ack_id)
            self.received_action_sequence_ids_condition.notify_all()

    # Called by handle_action_frame when an action response frame is received.
    def handle_action_response_frame(
        self, action_name: str, res_id: int, data: list
    ) -> None:
        if self.node.debug_info:
            self.node.get_logger().info(f"{action_name}#{res_id - 2} response")

        action_id = self.node.ordered_names.index(action_name)

        # Send one ACK frame.
        self.node.send_to_opposite_side([action_id, res_id + 1])  # ACK is always ID + 1

        # Ignore if no such request is pending.
        if res_id - 2 not in self.outgoing_action_goal_states:  # synced by GIL
            # This might happen when extra responses are sent after the call sequence was finalized and the first ACK did not arrive.
            # It is expected, so no error is logged.
            return

        # If this frame ID was not ACKed before...
        self.received_action_sequence_ids_condition.acquire()
        if res_id not in self.received_action_sequence_ids:
            # Save the frame ID as acknowledged.
            self.received_action_sequence_ids.add(res_id)
            self.received_action_sequence_ids_condition.notify_all()
            self.received_action_sequence_ids_condition.release()

            # Read the status.
            status = data[0]
            data = data[1:]  # Skip status.

            # Deserialize the response if status is SUCCEEDED.
            action_config = self.get_action_config(action_name)
            if status == GoalStatus.STATUS_SUCCEEDED:
                try:
                    result = deserialize_message(
                        data,
                        self.node.imported_interfaces[action_config["type"]].Result,
                        action_config["result_fields"],
                    )
                except ValueError as e:
                    self.node.get_logger().error(
                        f"Failed to deserialize a result of action {action_config['name']}:\n{e}"
                    )
                    return
                except Exception:
                    self.node.get_logger().error(
                        f"Unexpected error while deserializing a result of action {action_config['name']}:\n{traceback.format_exc()}"
                    )
                    return
            else:
                result = self.node.imported_interfaces[action_config["type"]].Result()

            # Save the response in the response dictionary.
            with self.outgoing_action_goal_states_condition:
                # response ID is request ID + 2
                self.outgoing_action_goal_states[res_id - 2].status = status
                self.outgoing_action_goal_states[res_id - 2].result = result
                self.outgoing_action_goal_states_condition.notify_all()
        try:
            self.received_action_sequence_ids_condition.release()
        except RuntimeError:
            pass

    # Called by handle_action_frame when an action response ACK frame is received.
    def handle_action_response_ack_frame(
        self, action_name: str, res_ack_id: int
    ) -> None:
        if self.node.debug_info:
            self.node.get_logger().info(f"{action_name}#{res_ack_id - 3} response ACK")

        # If request ID is not in received_action_sequence_ids, this ACK must correspond to a previous call, so ignore it.
        if res_ack_id - 3 not in self.received_action_sequence_ids:  # synced by GIL
            # This might happen when extra ACKs are sent after the action call sequence was finalized.
            # It is expected, so no error is logged.
            return

        with self.received_action_sequence_ids_condition:
            self.received_action_sequence_ids.add(res_ack_id)
            self.received_action_sequence_ids_condition.notify_all()

    # Called by handle_action_frame when an action feedback frame is received.
    def handle_action_feedback_frame(
        self, action_name: str, feedback_id: int, data: list
    ) -> None:
        if self.node.debug_info:
            self.node.get_logger().info(f"{action_name}#{feedback_id - 4} feedback")

        # Ignore if no such request is pending.
        if feedback_id - 4 not in self.outgoing_action_goal_states:  # synced by GIL
            # This might happen when extra feedbacks are sent after the call sequence was finalized and the first ACK did not arrive.
            # It is expected, so no error is logged.
            return

        # Deserialize the feedback.
        action_config = self.get_action_config(action_name)
        try:
            feedback = deserialize_message(
                data,
                self.node.imported_interfaces[action_config["type"]].Feedback,
                action_config["feedback_fields"],
            )
        except ValueError as e:
            self.node.get_logger().error(
                f"Failed to deserialize a feedback of action {action_config['name']}:\n{e}"
            )
            return
        except Exception:
            self.node.get_logger().error(
                f"Unexpected error while deserializing a feedback of action {action_config['name']}:\n{traceback.format_exc()}"
            )
            return

        # Save the feedback in the response dictionary.
        with self.outgoing_action_goal_states_condition:
            # feedback ID is request ID + 4
            self.outgoing_action_goal_states[feedback_id - 4].last_feedback = feedback
            self.outgoing_action_goal_states_condition.notify_all()

    # Called by handle_action_frame when an action cancel frame is received.
    def handle_action_cancel_frame(
        self, action_name: str, cancel_id: int, data: list
    ) -> None:
        if self.node.debug_info:
            self.node.get_logger().info(f"{action_name}#{cancel_id - 5} cancel")

        action_id = self.node.ordered_names.index(action_name)

        # If this frame ID was not received before...
        self.received_action_sequence_ids_condition.acquire()
        if cancel_id not in self.received_action_sequence_ids:
            # Save the frame ID as received.
            self.received_action_sequence_ids.add(cancel_id)
            self.received_action_sequence_ids_condition.notify_all()
            self.received_action_sequence_ids_condition.release()

            # If we have a goal handle for this request ID...
            with self.incoming_action_goal_states_lock:
                # Cancel ID is request ID + 5
                if cancel_id - 5 in self.incoming_action_goal_states:
                    state = self.incoming_action_goal_states[cancel_id - 5]
                    cancel_response: CancelGoal.Response = (
                        state.client_handle.cancel_goal()
                    )
                    goal_info: GoalInfo
                    for goal_info in cancel_response.goals_canceling:
                        if goal_id_to_hex_str(goal_info.goal_id) == goal_id_to_hex_str(
                            state.client_handle.goal_id
                        ):
                            state.cancelled = True
                            break
        try:
            self.received_action_sequence_ids_condition.release()
        except RuntimeError:
            pass

        # ACK the frame with optional cancelled=boolean attribute.
        with self.incoming_action_goal_states_lock:
            if cancel_id - 5 in self.incoming_action_goal_states:
                state = self.incoming_action_goal_states[cancel_id - 5]
                if state.cancelled:
                    self.node.send_to_opposite_side([action_id, cancel_id + 1, 1])
                else:
                    self.node.send_to_opposite_side([action_id, cancel_id + 1, 0])
            else:
                self.node.send_to_opposite_side([action_id, cancel_id + 1])

    # Called by handle_action_frame when an action cancel ACK frame is received.
    def handle_action_cancel_ack_frame(
        self, action_name: str, cancel_ack_id: int, data: list
    ) -> None:
        if self.node.debug_info:
            self.node.get_logger().info(f"{action_name}#{cancel_ack_id - 6} cancel ACK")

        # Ignore if no such request is pending.
        if cancel_ack_id - 6 not in self.outgoing_action_goal_states:  # synced by GIL
            # This might happen when extra cancel ACKs are sent after the call sequence was finalized.
            # It is expected, so no error is logged.
            return

        # Save whether the cancel was successful.
        if len(data) > 0:
            with self.outgoing_action_goal_states_condition:
                self.outgoing_action_goal_states[
                    cancel_ack_id - 6
                ].cancelled_state_from_ack = bool(data[0])
                self.outgoing_action_goal_states_condition.notify_all()

        # Save the ACK to received.
        with self.received_action_sequence_ids_condition:
            self.received_action_sequence_ids.add(cancel_ack_id)
            self.received_action_sequence_ids_condition.notify_all()

    # Called whenever an action request is received from this side.
    # The request is serialized and sent to the master.
    def execute_callback(
        self, goal_handle: ServerGoalHandle, action_config: dict
    ) -> Any:
        if self.node.debug_info:
            self.node.get_logger().info(
                f"Received goal for action {action_config['name']}."
            )

        # Determine the ID of the action.
        action_id = self.node.ordered_names.index(action_config["name"])

        # Serialize the request.
        try:
            data = serialize_message(goal_handle.request, action_config["goal_fields"])
        except ValueError as e:
            self.node.get_logger().error(
                f"Failed to serialize goal for action {action_config['name']}:\n{e}"
            )
            return
        except:
            self.node.get_logger().error(
                f"Unexpected error while serializing goal for action {action_config['name']}:\n{traceback.format_exc()}"
            )
            return

        # Generate an ID for this request.
        # The ID is unique for each call and is used to match responses to requests.
        with self.outgoing_action_req_id_counter_lock:
            # If the ID overflows over 1 byte, reset it.
            if self.outgoing_action_req_id_counter + 7 > 256:
                self.outgoing_action_req_id_counter = 0

            req_id = self.outgoing_action_req_id_counter
            # Increment the request ID by 4.
            # ID + 1 is reserved for request ACK.
            # ID + 2 is reserved for response.
            # ID + 3 is reserved for response ACK.
            self.outgoing_action_req_id_counter += 7

        # Clear all IDs reserved for this side from the received set.
        # action request >
        # request ACK <
        # action response <
        # response ACK >
        # action feedback <
        # action cancel >
        # cancel ACK <
        with self.received_action_sequence_ids_condition:
            if req_id + 1 in self.received_action_sequence_ids:
                self.received_action_sequence_ids.remove(req_id + 1)  # request ACK
            if req_id + 2 in self.received_action_sequence_ids:
                self.received_action_sequence_ids.remove(req_id + 2)  # response
            if req_id + 4 in self.received_action_sequence_ids:
                self.received_action_sequence_ids.remove(req_id + 4)  # feedback
            if req_id + 6 in self.received_action_sequence_ids:
                self.received_action_sequence_ids.remove(req_id + 6)  # cancel
            self.received_action_sequence_ids_condition.notify_all()
        # Clear outgoing_action_goal_states for this request ID.
        with self.outgoing_action_goal_states_condition:
            if req_id in self.outgoing_action_goal_states:
                self.outgoing_action_goal_states.pop(req_id)
                self.outgoing_action_goal_states_condition.notify_all()

        # Save the request ID as pending.
        self.outgoing_action_goal_states[req_id] = OutgoingGoalState(
            server_handle=goal_handle
        )  # synced by GIL

        if self.node.debug_info:
            self.node.get_logger().info(
                f"Assigned request ID {req_id} to goal {goal_id_to_hex_str(goal_handle.goal_id)}. Sending request frames until ACK..."
            )

        # Keep publishing the request frame until it is ACKed or until response is received.
        self.received_action_sequence_ids_condition.acquire()
        while (
            req_id + 1 not in self.received_action_sequence_ids
            and req_id + 2 not in self.received_action_sequence_ids
        ):  # Request ACK is ID + 1, response is ID + 2
            self.received_action_sequence_ids_condition.release()
            self.node.send_to_opposite_side([action_id, req_id] + data)
            self.received_action_sequence_ids_condition.acquire()
            self.received_action_sequence_ids_condition.wait(
                1 / action_config["retry_rate"]
            )
        self.received_action_sequence_ids_condition.release()

        # Wait for the response.
        # The response will be received in master_callback and saved to self.incoming_action_responses.
        with self.outgoing_action_goal_states_condition:
            while (
                req_id not in self.outgoing_action_goal_states
                or self.outgoing_action_goal_states[req_id].status is None
            ):
                # NOTE: Cancelled actions will receive status and response as usual. No need to check for cancellation here.
                if req_id in self.outgoing_action_goal_states:
                    # status is None, so the response is not ready yet, but there may be some feedback.
                    feedback = self.outgoing_action_goal_states[req_id].last_feedback
                    if feedback is not None:
                        goal_handle.publish_feedback(feedback)
                        self.outgoing_action_goal_states[req_id].last_feedback = None
                self.outgoing_action_goal_states_condition.wait()

        state = self.outgoing_action_goal_states[req_id]  # synced by GIL

        if self.node.debug_info:
            self.node.get_logger().info(
                f"Received response for action {action_config['name']}."
            )

        # Set goal status based on the response.
        if state.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif state.status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
        elif state.status == GoalStatus.STATUS_CANCELED:
            # Wait until cancel_callback exits. This can be checked with goal_handle.is_canceling().
            spin_start_time = time.time()
            spin_timed_out = False
            while not goal_handle.is_cancel_requested:
                if time.time() - spin_start_time > 10:
                    self.node.get_logger().error(
                        f"Unexpected error: Action {action_config['name']} was canceled on the opposite side, but this side did not set CancelResponse.ACCEPT within 10 seconds of receiving the response from opposite side. Aborting the action instead."
                    )
                    goal_handle.abort()
                    spin_timed_out = True
                    break
                time.sleep(0.01)
            if not spin_timed_out:
                goal_handle.canceled()
        else:
            self.node.get_logger().error(
                f"Received an unsupported status {state.status} for action {action_config['name']}. Falling back to ABORTED."
            )
            goal_handle.abort()

        return state.result

    # Called by ROS whenever an action is canceled.
    def cancel_callback(
        self, goal_handle: ServerGoalHandle, action_config: dict
    ) -> CancelResponse:
        if self.node.debug_info:
            self.node.get_logger().info(
                f"Received cancel request for action {action_config['name']}, goal ID {goal_id_to_hex_str(goal_handle.goal_id)}."
            )

        # Determine the ID of the action.
        action_id = self.node.ordered_names.index(action_config["name"])

        # Check if this goal handle is pending.
        req_id: int | None = None
        with self.outgoing_action_goal_states_condition:
            for req_id, state in self.outgoing_action_goal_states.items():
                if goal_id_to_hex_str(
                    state.server_handle.goal_id
                ) == goal_id_to_hex_str(goal_handle.goal_id):
                    # Get the request ID for this goal handle.
                    req_id = req_id
                    break

        if req_id is None:
            self.node.get_logger().error(
                f"Received a cancel request for an action that is not pending. Rejecting this request."
            )
            return CancelResponse.REJECT

        # Start sending cancel frames until ACK is received.
        self.node.get_logger().info(
            f"Sending cancel requests for action {action_config['name']}#{req_id} until ACK..."
        )
        self.received_action_sequence_ids_condition.acquire()
        while req_id + 6 not in self.received_action_sequence_ids:
            self.received_action_sequence_ids_condition.release()
            self.node.send_to_opposite_side([action_id, req_id + 5])
            self.received_action_sequence_ids_condition.acquire()
            self.received_action_sequence_ids_condition.wait(
                1 / action_config["retry_rate"]
            )
        self.received_action_sequence_ids_condition.release()

        self.node.get_logger().info(
            f"Received cancel ACK for action {action_config['name']}#{req_id} returning cancel response."
        )

        # Return whether the cancel was successful.
        with self.outgoing_action_goal_states_condition:
            return (
                CancelResponse.ACCEPT
                if self.outgoing_action_goal_states[req_id].cancelled_state_from_ack
                else CancelResponse.REJECT
            )
