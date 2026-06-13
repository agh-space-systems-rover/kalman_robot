from __future__ import annotations

import json
import os
import time
import urllib.request
import uuid

import streamlit as st

from kalman_tools.master_message_catalog import (
    format_cmd,
    format_payload,
    format_payload_hex,
    frames_for_cmd,
    list_frame_definitions,
    parse_cmd,
    parse_payload,
)
from kalman_tools.ros_bridge import Filter, MasterRosBridge

FILTER_OPS = ["==", "!=", "<", ">", "<=", ">="]
DEFAULT_SEND_ROW = {
    "name": "",
    "cmd_text": "AUTONOMY_SWITCH",
    "payload_text": "1",
    "interval_s": 1.0,
}
DEBUG_LOG_PATH = "/home/wiktor/programming/students_research_group/agh_space_systems/kalman_ws/.cursor/debug-cebf79.log"
DEBUG_SESSION_ID = "cebf79"
DEBUG_INGEST_URL = "http://127.0.0.1:7506/ingest/07c2b1c4-cc73-4d5a-b1d6-172fb49d8897"


def _debug_log(hypothesis_id: str, location: str, message: str, data: dict) -> None:
    payload = {
        "sessionId": DEBUG_SESSION_ID,
        "runId": f"streamlit-session-{id(st.session_state)}",
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    try:
        request = urllib.request.Request(
            DEBUG_INGEST_URL,
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "X-Debug-Session-Id": DEBUG_SESSION_ID,
            },
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=0.2):
            pass
    except Exception:
        pass
    try:
        with open(DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
            debug_file.write(json.dumps(payload, separators=(",", ":")) + "\n")
    except Exception:
        pass


def _init_session_state() -> None:
    if "send_rows" not in st.session_state:
        st.session_state.send_rows = [{**DEFAULT_SEND_ROW, "id": str(uuid.uuid4())}]
    if "receive_cmds" not in st.session_state:
        st.session_state.receive_cmds = []
    if "receive_filters" not in st.session_state:
        st.session_state.receive_filters = []
    if "bridge_key" not in st.session_state:
        st.session_state.bridge_key = None


def _get_bridge(host: str, port: int) -> MasterRosBridge:
    key = (host, port)
    bridge = st.session_state.get("bridge")
    # region agent log
    _debug_log(
        "H2",
        "master_frame_panel.py:66",
        "_get_bridge called",
        {
            "requestedKey": list(key),
            "existingBridgeId": id(bridge) if bridge is not None else None,
            "existingKey": list(st.session_state.get("bridge_key"))
            if st.session_state.get("bridge_key") is not None
            else None,
        },
    )
    # endregion
    if bridge is None or st.session_state.get("bridge_key") != key:
        if bridge is not None:
            # region agent log
            _debug_log(
                "H2",
                "master_frame_panel.py:80",
                "closing previous bridge before recreate",
                {
                    "oldBridgeId": id(bridge),
                    "oldKey": list(st.session_state.get("bridge_key"))
                    if st.session_state.get("bridge_key") is not None
                    else None,
                    "newKey": list(key),
                },
            )
            # endregion
            bridge.close()
        bridge = MasterRosBridge(host=host, port=port)
        st.session_state.bridge = bridge
        st.session_state.bridge_key = key
    return bridge


@st.cache_resource
def _get_cached_bridge() -> MasterRosBridge:
    host = os.environ.get("KALMAN_TOOLS_ROSBRIDGE_HOST", "localhost")
    port = int(os.environ.get("KALMAN_TOOLS_ROSBRIDGE_PORT", "3001"))
    # region agent log
    _debug_log(
        "H2",
        "master_frame_panel.py:106",
        "creating cached bridge",
        {
            "host": host,
            "port": port,
        },
    )
    # endregion
    return MasterRosBridge(host=host, port=port)


def _catalog_labels() -> list[str]:
    return [frame.label for frame in list_frame_definitions()]


def _label_to_cmd_text(label: str) -> str:
    for frame in list_frame_definitions():
        if frame.label == label:
            return frame.name
    return label


def _set_cmd_from_catalog(row_id: str, catalog_labels: list[str]) -> None:
    label = st.session_state.get(f"send_catalog_{row_id}")
    if label in catalog_labels:
        st.session_state[f"send_cmd_{row_id}"] = _label_to_cmd_text(label)


def _catalog_index_for_cmd_text(cmd_text: str, catalog_labels: list[str]) -> int:
    for frame in list_frame_definitions():
        if frame.name == cmd_text:
            try:
                return catalog_labels.index(frame.label)
            except ValueError:
                break
    return 0


def _cmd_names_for_display(cmd: int) -> str:
    names = [frame.name for frame in frames_for_cmd(cmd)]
    return ", ".join(names) if names else format_cmd(cmd)


def _render_sidebar(bridge: MasterRosBridge) -> float:
    st.sidebar.header("Connection")
    st.sidebar.text_input("Rosbridge host", value=bridge.host, disabled=True)
    st.sidebar.number_input(
        "Rosbridge port",
        min_value=1,
        max_value=65535,
        value=int(bridge.port),
        help="Ground station rosbridge uses port 9065.",
        disabled=True,
    )
    st.sidebar.caption(
        "Host/port fixed per Streamlit process. Restart app to change connection target."
    )
    if bridge.connected:
        st.sidebar.success("Connected to rosbridge")
    else:
        st.sidebar.warning("Waiting for rosbridge...")

    refresh_ms = st.sidebar.number_input(
        "Receive auto-refresh (ms, 0 = off)",
        min_value=0,
        max_value=10000,
        value=500,
        step=100,
    )
    return float(refresh_ms)


def _render_send_tab(bridge: MasterRosBridge) -> None:
    st.subheader("Send frames")
    st.caption("Frames are published to `master_com/ros_to_master`.")

    catalog_labels = _catalog_labels()
    rows = st.session_state.send_rows

    if st.button("Add frame row", key="add_send_row"):
        rows.append({**DEFAULT_SEND_ROW, "id": str(uuid.uuid4())})

    remove_ids: list[str] = []
    for index, row in enumerate(rows):
        row_id = row["id"]
        with st.container(border=True):
            cols = st.columns([2, 2, 3, 1])
            with cols[0]:
                row["name"] = st.text_input(
                    "Label",
                    value=row.get("name", ""),
                    key=f"send_name_{row_id}",
                )
            with cols[1]:
                st.selectbox(
                    "Frame",
                    options=catalog_labels,
                    index=_catalog_index_for_cmd_text(
                        st.session_state.get(
                            f"send_cmd_{row_id}", row.get("cmd_text", "")
                        ),
                        catalog_labels,
                    ),
                    key=f"send_catalog_{row_id}",
                    on_change=_set_cmd_from_catalog,
                    args=(row_id, catalog_labels),
                )
            with cols[2]:
                cmd_text = st.text_input(
                    "Cmd (name, 0xNN, or decimal)",
                    value=row.get("cmd_text", ""),
                    key=f"send_cmd_{row_id}",
                )
                row["cmd_text"] = cmd_text
                payload_text = st.text_input(
                    "Payload bytes",
                    value=row.get("payload_text", ""),
                    key=f"send_payload_{row_id}",
                    help="Comma or space separated values, e.g. 1, 0, 255",
                )
                row["payload_text"] = payload_text
            with cols[3]:
                row["interval_s"] = st.number_input(
                    "Spam interval (s)",
                    min_value=0.05,
                    value=float(row.get("interval_s", 1.0)),
                    step=0.05,
                    key=f"send_interval_{row_id}",
                )
                if st.button("Remove", key=f"remove_send_{row_id}"):
                    remove_ids.append(row_id)

            action_cols = st.columns(4)
            parse_error = None
            cmd = None
            data = None
            try:
                cmd = parse_cmd(row["cmd_text"])
                data = parse_payload(row["payload_text"])
            except ValueError as exc:
                parse_error = str(exc)

            if parse_error:
                st.error(parse_error)
            else:
                st.text(
                    f"Topic preview: master_com/ros_to_master  |  cmd={format_cmd(cmd)}  data={format_payload(data)}"
                )

            with action_cols[0]:
                if st.button("Send now", key=f"send_now_{row_id}", disabled=parse_error is not None):
                    try:
                        bridge.send_frame(cmd, data)
                        st.success("Frame sent")
                    except RuntimeError as exc:
                        st.error(str(exc))
            with action_cols[1]:
                if st.button(
                    "Start spam",
                    key=f"start_spam_{row_id}",
                    disabled=parse_error is not None,
                ):
                    try:
                        bridge.start_spam(row_id, cmd, data, float(row["interval_s"]))
                        st.info(f"Spamming every {row['interval_s']} s")
                    except (RuntimeError, ValueError) as exc:
                        st.error(str(exc))
            with action_cols[2]:
                if st.button("Stop spam", key=f"stop_spam_{row_id}"):
                    bridge.stop_spam(row_id)
                    st.info("Spam stopped")
            with action_cols[3]:
                if st.button("Duplicate", key=f"duplicate_send_{row_id}"):
                    rows.append(
                        {
                            **row,
                            "id": str(uuid.uuid4()),
                        }
                    )

    if remove_ids:
        st.session_state.send_rows = [
            row for row in rows if row["id"] not in remove_ids
        ]
        for row_id in remove_ids:
            bridge.stop_spam(row_id)

    if st.button("Stop all spam", type="primary"):
        bridge.stop_all_spam()
        st.info("All spam stopped")


def _render_receive_tab(bridge: MasterRosBridge, refresh_ms: float) -> None:
    st.subheader("Receive frames")
    st.caption("Subscribe to `master_com/master_to_ros/xNN` topics.")

    frames = list_frame_definitions()
    label_to_cmd = {frame.label: frame.cmd for frame in frames}
    selected_labels = st.multiselect(
        "Frame IDs to display",
        options=[frame.label for frame in frames],
        default=st.session_state.receive_cmds,
        help="Multiple names can share the same cmd value.",
    )
    st.session_state.receive_cmds = selected_labels
    selected_cmds = {label_to_cmd[label] for label in selected_labels}

    if selected_labels:
        topic_lines = []
        seen_cmds: set[int] = set()
        for label in selected_labels:
            cmd = label_to_cmd[label]
            if cmd in seen_cmds:
                continue
            seen_cmds.add(cmd)
            topic_lines.append(
                f"{format_cmd(cmd)} -> master_com/master_to_ros/{format_cmd(cmd)[1:]}"
            )
        st.text("Subscribed topics:\n" + "\n".join(topic_lines))

    st.markdown("**Data filters** (all must match)")
    filter_rows = st.session_state.receive_filters
    if st.button("Add filter"):
        filter_rows.append({"index": 0, "op": "==", "value": 0})

    remove_filter_indexes: list[int] = []
    parsed_filters: list[Filter] = []
    for index, filt in enumerate(filter_rows):
        cols = st.columns([1, 1, 1, 1])
        filt["index"] = cols[0].number_input(
            "Index",
            min_value=0,
            value=int(filt.get("index", 0)),
            key=f"filter_index_{index}",
        )
        filt["op"] = cols[1].selectbox(
            "Operator",
            options=FILTER_OPS,
            index=FILTER_OPS.index(filt.get("op", "==")),
            key=f"filter_op_{index}",
        )
        filt["value"] = cols[2].number_input(
            "Value",
            min_value=0,
            max_value=255,
            value=int(filt.get("value", 0)),
            key=f"filter_value_{index}",
        )
        if cols[3].button("Remove", key=f"remove_filter_{index}"):
            remove_filter_indexes.append(index)
        else:
            parsed_filters.append(
                Filter(
                    index=int(filt["index"]),
                    op=str(filt["op"]),
                    value=int(filt["value"]),
                )
            )

    if remove_filter_indexes:
        st.session_state.receive_filters = [
            filt
            for idx, filt in enumerate(filter_rows)
            if idx not in remove_filter_indexes
        ]
        parsed_filters = [
            Filter(
                index=int(filt["index"]),
                op=str(filt["op"]),
                value=int(filt["value"]),
            )
            for filt in st.session_state.receive_filters
        ]

    bridge.set_subscriptions(selected_cmds, parsed_filters)

    control_cols = st.columns(2)
    if control_cols[0].button("Clear log"):
        bridge.clear_received_messages()
    if control_cols[1].button("Refresh now"):
        st.rerun()

    if refresh_ms > 0:
        st.markdown(
            f"<meta http-equiv='refresh' content='{refresh_ms / 1000:.3f}'>",
            unsafe_allow_html=True,
        )

    messages = bridge.get_received_messages()
    if bridge.connected and selected_cmds and not messages:
        st.info(
            "No messages yet. Ensure `master_com` (or `master_loopback`) is running and sending matching frames."
        )

    if not messages:
        st.write("No received frames.")
        return

    table_rows = []
    for message in reversed(messages):
        local_time = message.time.astimezone().strftime("%H:%M:%S.%f")[:-3]
        table_rows.append(
            {
                "time": local_time,
                "cmd": _cmd_names_for_display(message.cmd),
                "topic": message.topic,
                "data_dec": format_payload(message.data),
                "data_hex": format_payload_hex(message.data),
            }
        )
    st.dataframe(table_rows, use_container_width=True, hide_index=True)


def main() -> None:
    st.set_page_config(
        page_title="Master Frame Panel",
        page_icon="📡",
        layout="wide",
    )
    st.title("Master Frame Panel")
    st.caption("Send and inspect Master protocol frames via rosbridge.")

    _init_session_state()

    bridge = _get_cached_bridge()

    refresh_ms = _render_sidebar(bridge)

    send_tab, receive_tab = st.tabs(["Send", "Receive"])
    with send_tab:
        _render_send_tab(bridge)
    with receive_tab:
        _render_receive_tab(bridge, refresh_ms)


if __name__ == "__main__":
    main()
