from __future__ import annotations

import traceback
import time
from threading import Lock, Condition

from kalman_master.ros_link_serialization import serialize_message, deserialize_message

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from kalman_master.ros_link_node import RosLink
    from rclpy.client import Client


class RosLinkServices:
    def __init__(self, node: RosLink):
        self.node = node

        # Create clients for services provided by this side.
        self.service_clients: dict[str, Client] = {}
        for service_config in self.node.get_this_side_config()["services"]:
            self.service_clients[service_config["name"]] = self.node.create_client(
                self.node.imported_interfaces[service_config["type"]],
                service_config["name"],
            )

        # Create services provided by the opposite side.
        for service_config in self.node.get_opposite_side_config()["services"]:
            self.node.create_service(
                self.node.imported_interfaces[service_config["type"]],
                service_config["name"]
                + ("/loopback" if self.node.loopback_mangling else ""),
                lambda req, res, service_config=service_config: self.service_callback(
                    req, res, service_config
                ),
                callback_group=self.node.callback_group,
            )

        # Generates quadruple ID sequences for unique service calls. IDs are generated in service_callback.
        self.outgoing_service_req_id_counter = 0
        self.outgoing_service_req_id_counter_lock = Lock()
        # Used to ignore request ACKs for requests that are not pending. A call to a service on the opposite side is pending if its request ID is in this set.
        self.pending_outgoing_service_req_ids = set()
        self.pending_outgoing_service_req_ids_lock = Lock()
        # Used to ignore duplicate request/response frames and to check whether an ACK was received. Also used to ignore response ACKs for calls that are not pending anymore. A call to service on this side is pending if its request ID is in this set.
        self.received_service_sequence_ids = set()
        self.received_service_sequence_ids_condition = Condition()
        # Used to store responses from services hosted on the opposite side. The responses are received by master_callback, but they need to be passed to another thread that is waiting in service_callback. Keys are request IDs, values are deserialized response messages.
        self.incoming_service_responses = {}
        self.incoming_service_responses_condition = Condition()

    # Get the configuration for a service.
    def get_service_config(self, name: str) -> dict:
        for side_config in [
            self.node.get_this_side_config(),
            self.node.get_opposite_side_config(),
        ]:
            for service_config in side_config["services"]:
                if service_config["name"] == name:
                    return service_config
        raise ValueError(f"No configuration found for service {name}.")

    # Called by master_callback when a service frame is received.
    def handle_service_frame(self, service_name: str, data: list) -> None:
        # Skip if the frame does not contain sequence ID.
        if len(data) < 1:
            self.node.get_logger().error(
                f"Master delivered a service frame without sequence ID for service {service_name}."
            )
            return

        seq_id = data[0]
        data = data[1:]  # Skip sequence ID.

        # If this is a request frame...
        if seq_id % 4 == 0:
            self.handle_service_request_frame(service_name, seq_id, data)
        # If this is a request ACK frame...
        elif seq_id % 4 == 1:
            # Ignore if no such request is pending.
            if seq_id - 1 not in self.pending_outgoing_service_req_ids:  # synced by GIL
                # This might happen when extra request ACKs are sent after the call sequence was finalized.
                return
            with self.received_service_sequence_ids_condition:
                self.received_service_sequence_ids.add(seq_id)
                self.received_service_sequence_ids_condition.notify_all()
        # If this is a response frame...
        elif seq_id % 4 == 2:
            self.handle_service_response_frame(service_name, seq_id, data)
        # If this is a response ACK frame...
        elif seq_id % 4 == 3:
            # If request ID is not in received_service_sequence_ids, this ACK must correspond to a previous call, so ignore it.
            if seq_id - 3 not in self.received_service_sequence_ids:  # synced by GIL
                # This might happen when extra ACKs are sent after the service call sequence was finalized.
                # It is expected, so no error is logged.
                return
            with self.received_service_sequence_ids_condition:
                self.received_service_sequence_ids.add(seq_id)
                self.received_service_sequence_ids_condition.notify_all()

    # Called by handle_service_frame when a service request frame is received.
    def handle_service_request_frame(
        self, service_name: str, req_id: int, data: list
    ) -> None:
        service_id = self.node.ordered_names.index(service_name)

        # Send one ACK frame.
        self.node.send_to_opposite_side(
            [service_id, req_id + 1]
        )  # ACK is always ID + 1

        # If this frame ID was not ACKed before...
        self.received_service_sequence_ids_condition.acquire()
        if req_id not in self.received_service_sequence_ids:
            # Save the frame ID as acknowledged.
            self.received_service_sequence_ids.add(req_id)
            self.received_service_sequence_ids_condition.notify_all()
            self.received_service_sequence_ids_condition.release()

            # Deserialize the request.
            service_config = self.get_service_config(service_name)
            try:
                req = deserialize_message(
                    data,
                    self.node.imported_interfaces[service_config["type"]].Request,
                    service_config["request_fields"],
                )
            except ValueError as e:
                self.node.get_logger().error(
                    f"Failed to deserialize a request to service {service_config['name']}:\n{e}"
                )
                return
            except Exception:
                self.node.get_logger().error(
                    f"Unexpected error while deserializing a request to service {service_config['name']}:\n{traceback.format_exc()}"
                )
                return

            # Call the service.
            last_waiting_for_service_log_time = 0
            while not self.service_clients[service_name].service_is_ready():
                if time.time() - last_waiting_for_service_log_time > 5:
                    self.node.get_logger().info(
                        f"Waiting for service {service_name}..."
                    )
                    last_waiting_for_service_log_time = time.time()
                time.sleep(
                    0.01
                )  # TODO: Maybe support cancelling the action while waiting for the server to be ready? Not sure if it is very useful though.

            res = self.service_clients[service_name].call(req)

            # Serialize the response.
            try:
                data = serialize_message(res, service_config["response_fields"])
            except ValueError as e:
                self.node.get_logger().error(
                    f"Failed to serialize response to service {service_config['name']}:\n{e}"
                )
                return
            except Exception:
                self.node.get_logger().error(
                    f"Unexpected error while serializing a response to service {service_config['name']}:\n{traceback.format_exc()}"
                )
                return

            # Keep publishing the response frame until it is ACKed.
            self.received_service_sequence_ids_condition.acquire()
            while (
                req_id + 3 not in self.received_service_sequence_ids
            ):  # Response ACK is request ID + 3
                self.received_service_sequence_ids_condition.release()
                self.node.send_to_opposite_side(
                    [service_id, req_id + 2] + data
                )  # Response is request ID + 2
                self.received_service_sequence_ids_condition.acquire()
                self.received_service_sequence_ids_condition.wait(
                    1 / service_config["retry_rate"]
                )
            self.received_service_sequence_ids_condition.release()

            # The service call is done, forget the IDs used by this side.
            with self.received_service_sequence_ids_condition:
                self.received_service_sequence_ids.remove(req_id)  # request ID
                self.received_service_sequence_ids.remove(req_id + 3)  # response ACK ID
        try:
            self.received_service_sequence_ids_condition.release()
        except RuntimeError:
            pass

    # Called by handle_service_frame when a service response frame is received.
    def handle_service_response_frame(
        self, service_name: str, res_id: int, data: list
    ) -> None:
        service_id = self.node.ordered_names.index(service_name)

        # Send one ACK frame.
        self.node.send_to_opposite_side(
            [service_id, res_id + 1]
        )  # ACK is always ID + 1

        # Ignore if no such request is pending.
        if res_id - 2 not in self.pending_outgoing_service_req_ids:  # synced by GIL
            # This might happen when extra responses are sent after the call sequence was finalized and the first ACK did not arrive.
            # It is expected, so no error is logged.
            return

        # If this frame ID was not ACKed before...
        self.received_service_sequence_ids_condition.acquire()
        if res_id not in self.received_service_sequence_ids:
            # Save the frame ID as acknowledged.
            self.received_service_sequence_ids.add(res_id)
            self.received_service_sequence_ids_condition.notify_all()
            self.received_service_sequence_ids_condition.release()

            # Deserialize the response.
            service_config = self.get_service_config(service_name)
            try:
                res = deserialize_message(
                    data,
                    self.node.imported_interfaces[service_config["type"]].Response,
                    service_config["response_fields"],
                )
            except ValueError as e:
                self.node.get_logger().error(
                    f"Failed to deserialize a response to service {service_config['name']}:\n{e}"
                )
                return
            except Exception:
                self.node.get_logger().error(
                    f"Unexpected error while deserializing a response to service {service_config['name']}:\n{traceback.format_exc()}"
                )
                return

            # Save the response in a dictionary.
            with self.incoming_service_responses_condition:
                self.incoming_service_responses[res_id - 2] = (
                    res  # response ID is request ID + 2
                )
                self.incoming_service_responses_condition.notify_all()
        try:
            self.received_service_sequence_ids_condition.release()
        except RuntimeError:
            pass

    # Called whenever a service request is received from this side.
    # The request is serialized and sent to the master.
    def service_callback(self, req, res, service_config: dict) -> Any:
        # Determine the ID of the service.
        service_id = self.node.ordered_names.index(service_config["name"])

        # Serialize the request.
        try:
            data = serialize_message(req, service_config["request_fields"])
        except ValueError as e:
            self.node.get_logger().error(
                f"Failed to serialize request for service {service_config['name']}:\n{e}"
            )
            return res
        except:
            self.node.get_logger().error(
                f"Unexpected error while serializing a request for service {service_config['name']}:\n{traceback.format_exc()}"
            )
            return res

        # Generate an ID for this request.
        # The ID is unique for each call and is used to match responses to requests.
        with self.outgoing_service_req_id_counter_lock:
            # If the ID overflows over 1 byte, reset it.
            if self.outgoing_service_req_id_counter + 4 > 256:
                self.outgoing_service_req_id_counter = 0

            req_id = self.outgoing_service_req_id_counter
            # Increment the request ID by 4.
            # ID + 1 is reserved for request ACK.
            # ID + 2 is reserved for response.
            # ID + 3 is reserved for response ACK.
            self.outgoing_service_req_id_counter += 4

        # Save the request ID as pending.
        self.pending_outgoing_service_req_ids.add(req_id)  # synced by GIL

        # Keep publishing the request frame until it is ACKed or until response is received.
        self.received_service_sequence_ids_condition.acquire()
        while (
            req_id + 1 not in self.received_service_sequence_ids
            and req_id + 2 not in self.received_service_sequence_ids
        ):  # Request ACK is ID + 1, response is ID + 2
            self.received_service_sequence_ids_condition.release()
            self.node.send_to_opposite_side([service_id, req_id] + data)
            self.received_service_sequence_ids_condition.acquire()
            self.received_service_sequence_ids_condition.wait(
                1 / service_config["retry_rate"]
            )
        self.received_service_sequence_ids_condition.release()

        # Wait for the response.
        # The response will be received in master_callback and saved to self.incoming_service_responses.
        with self.incoming_service_responses_condition:
            while req_id not in self.incoming_service_responses:
                self.incoming_service_responses_condition.wait()

        # Clear the request ID from pending.
        self.pending_outgoing_service_req_ids.remove(req_id)  # synced by GIL

        # Clear the request sequence IDs used by this side.
        with self.received_service_sequence_ids_condition:
            if (
                req_id + 1 in self.received_service_sequence_ids
            ):  # request ACK ID; It might not have arrived if the response was received before the ACK.
                self.received_service_sequence_ids.remove(req_id + 1)
            self.received_service_sequence_ids.remove(req_id + 2)  # response ID

        return self.incoming_service_responses.pop(req_id)
