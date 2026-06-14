from __future__ import annotations

import os
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


@st.cache_resource
def _get_bridge() -> MasterRosBridge:
    host = os.environ.get("KALMAN_TOOLS_ROSBRIDGE_HOST", "localhost")
    port = int(os.environ.get("KALMAN_TOOLS_ROSBRIDGE_PORT", "3001"))
    return MasterRosBridge(host=host, port=port)


def _init_session_state() -> None:
    if "send_rows" not in st.session_state:
        st.session_state.send_rows = [{**DEFAULT_SEND_ROW, "id": str(uuid.uuid4())}]
    if "receive_cmds" not in st.session_state:
        st.session_state.receive_cmds = []
    if "receive_filters" not in st.session_state:
        st.session_state.receive_filters = []


def _catalog_labels() -> list[str]:
    return [frame.label for frame in list_frame_definitions()]


def _label_to_name(label: str) -> str:
    for frame in list_frame_definitions():
        if frame.label == label:
            return frame.name
    return label


def _catalog_index(cmd_text: str, catalog_labels: list[str]) -> int:
    for frame in list_frame_definitions():
        if frame.name == cmd_text:
            try:
                return catalog_labels.index(frame.label)
            except ValueError:
                break
    return 0


def _on_catalog_change(row_id: str, catalog_labels: list[str]) -> None:
    label = st.session_state.get(f"send_catalog_{row_id}", "")
    if label in catalog_labels:
        st.session_state[f"send_cmd_{row_id}"] = _label_to_name(label)


def _cmd_display(cmd: int) -> str:
    names = [frame.name for frame in frames_for_cmd(cmd)]
    return ", ".join(names) if names else format_cmd(cmd)


def _render_sidebar(bridge: MasterRosBridge) -> float:
    st.sidebar.header("Connection Settings")
    st.sidebar.text_input("Host Address", value=bridge.host, disabled=True)
    st.sidebar.number_input("Port Mapping", value=int(bridge.port), disabled=True)

    if bridge.connected:
        st.sidebar.success("Node Status: Online")
    else:
        st.sidebar.warning("Node Status: Resolving Link...")

    refresh_ms = st.sidebar.number_input(
        "Telemetry Refresh Rate (ms, 0=Off)",
        min_value=0,
        max_value=10000,
        value=500,
        step=100,
    )
    return float(refresh_ms)


def _render_send_tab(bridge: MasterRosBridge) -> None:
    st.subheader("Transmit Commands")
    st.caption("Dispatches payload envelopes onto `/master_com/ros_to_master`.")

    catalog_labels = _catalog_labels()
    rows: list[dict] = st.session_state.send_rows

    if st.button("＋ Append Frame Sequence", key="add_send_row", use_container_width=True):
        rows.append({**DEFAULT_SEND_ROW, "id": str(uuid.uuid4())})
        st.rerun()

    remove_ids: list[str] = []
    duplicate_rows: list[dict] = []

    for row in list(rows):
        row_id = row["id"]
        spamming = bridge.is_spamming(row_id)

        with st.container(border=True):
            hcols = st.columns([5, 1, 1])
            with hcols[0]:
                row["name"] = st.text_input("Label / Operational Alias", key=f"send_name_{row_id}", disabled=spamming)
            with hcols[1]:
                if st.button("🗑", key=f"remove_send_{row_id}", help="Drop Sequence", use_container_width=True):
                    remove_ids.append(row_id)
                    bridge.stop_spam(row_id)
            with hcols[2]:
                if st.button("⎘", key=f"dup_send_{row_id}", help="Clone Sequence", use_container_width=True):
                    duplicate_rows.append({**row, "id": str(uuid.uuid4())})

            fcols = st.columns([1, 1])
            with fcols[0]:
                st.selectbox(
                    "Catalog Blueprint Lookup",
                    options=catalog_labels,
                    index=_catalog_index(st.session_state.get(f"send_cmd_{row_id}", row.get("cmd_text", "")), catalog_labels),
                    key=f"send_catalog_{row_id}",
                    on_change=_on_catalog_change,
                    args=(row_id, catalog_labels),
                    disabled=spamming,
                )
            with fcols[1]:
                cmd_text: str = st.text_input("Command Register Identifier", key=f"send_cmd_{row_id}", disabled=spamming)

            pcols = st.columns([3, 1])
            with pcols[0]:
                payload_text: str = st.text_input("Data Payload Array (uint8)", key=f"send_payload_{row_id}", disabled=spamming)
            with pcols[1]:
                interval_s: float = st.number_input(
                    "Spam Frequency (s)", min_value=0.05, value=float(row.get("interval_s", 1.0)), step=0.05, key=f"send_interval_{row_id}", disabled=spamming
                )

            # Parse input parameters safely
            cmd_val, data_val, parse_error = None, None, None
            try:
                cmd_val = parse_cmd(cmd_text)
                data_val = parse_payload(payload_text)
            except Exception as exc:
                parse_error = str(exc)

            if parse_error:
                st.error(f"Formatting Violation: {parse_error}")
                disabled_action = True
            else:
                st.caption(f"Parsed Preview: `cmd={format_cmd(cmd_val)}` | `data=[{format_payload(data_val)}]` ")
                disabled_action = not bridge.connected

            acols = st.columns([1, 1, 2])
            with acols[0]:
                if st.button("Send Once", key=f"send_now_{row_id}", disabled=disabled_action, type="primary", use_container_width=True):
                    try:
                        bridge.send_frame(cmd_val, data_val)  # type: ignore[arg-type]
                        st.toast("Single frame written to bus.", icon="✅")
                    except RuntimeError as exc:
                        st.error(str(exc))

            with acols[1]:
                if not spamming:
                    if st.button("▶ Start Spam", key=f"start_spam_{row_id}", disabled=disabled_action, use_container_width=True):
                        try:
                            bridge.start_spam(row_id, cmd_val, data_val, interval_s)  # type: ignore[arg-type]
                            st.toast("Continuous scheduler engaged.", icon="📡")
                            st.rerun()
                        except Exception as exc:
                            st.error(str(exc))
                else:
                    if st.button("⏹ Stop Spam", key=f"stop_spam_{row_id}", type="primary", use_container_width=True):
                        bridge.stop_spam(row_id)
                        st.rerun()

            with acols[2]:
                if spamming:
                    st.markdown("<div style='padding-top:25px; color:#f0a500; font-weight:bold;'>● Transmitting...</div>", unsafe_allow_html=True)

            row["interval_s"] = interval_s

    if remove_ids:
        st.session_state.send_rows = [r for r in rows if r["id"] not in remove_ids]
        st.rerun()
    if duplicate_rows:
        st.session_state.send_rows = rows + duplicate_rows
        st.rerun()

    st.divider()
    if st.button("⏹ Emergency Intercept: Cease All Spams", type="primary", key="stop_all_spam", use_container_width=True):
        bridge.stop_all_spam()
        st.rerun()


# Wrap the entire logs terminal layout inside an isolated Streamlit Fragment
@st.fragment
def _render_isolated_log_terminal(bridge: MasterRosBridge, selected_cmds: set[int], refresh_ms: float) -> None:
    """Reruns strictly this function block at specified refresh intervals, matching optimal UX."""
    ctrl = st.columns([1, 1, 4])
    if ctrl[0].button("🗑 Flush Buffer", key="clear_log", use_container_width=True):
        bridge.clear_received_messages()
        st.rerun()
    if ctrl[1].button("⟳ Manual Sync", key="refresh_log", use_container_width=True):
        st.rerun()

    messages = bridge.get_received_messages()

    if bridge.connected and selected_cmds and not messages:
        st.info("Awaiting bus frames. Ensure telemetry transmitters or loopbacks are activated.")

    if not messages:
        st.write("Frame buffer empty.")
        return

    rows_data = []
    for msg in reversed(messages):
        local_time = msg.time.astimezone().strftime("%H:%M:%S.%f")[:-3]
        rows_data.append(
            {
                "Timestamp": local_time,
                "Command Directive": _cmd_display(msg.cmd),
                "Destination Topic": msg.topic,
                "Data (Dec)": format_payload(msg.data),
                "Data (Hex)": format_payload_hex(msg.data),
            }
        )
    st.dataframe(rows_data, use_container_width=True, hide_index=True)

    # Re-trigger only this fragment without causing global text-box resets or focus losses
    if refresh_ms > 0:
        st.cache_resource.get_id() # Keep dynamic fragment bindings alive
        time_s = refresh_ms / 1000.0
        st.rerun(scope="fragment")


def _render_receive_tab(bridge: MasterRosBridge, refresh_ms: float) -> None:
    st.subheader("Bus Monitor Terminal")
    st.caption("Eavesdrops on downstream command topics matching `master_com/master_to_ros/xNN`.")

    frames = list_frame_definitions()
    label_to_cmd = {frame.label: frame.cmd for frame in frames}

    selected_labels: list[str] = st.multiselect(
        "Selective Filtering: Display IDs",
        options=[frame.label for frame in frames],
        default=st.session_state.receive_cmds,
    )
    st.session_state.receive_cmds = selected_labels
    selected_cmds = {label_to_cmd[lbl] for lbl in selected_labels}

    # Handle Filter Conditions safely
    st.markdown("**Logical Payload Filters** (All conditions must resolve — AND Constraint)")
    filter_rows: list[dict] = st.session_state.receive_filters
    if st.button("＋ Inject Logic Filter", key="add_filter"):
        filter_rows.append({"index": 0, "op": "==", "value": 0})
        st.rerun()

    remove_filter_idxs: list[int] = []
    parsed_filters: list[Filter] = []
    for i, filt in enumerate(filter_rows):
        fcols = st.columns([1, 1, 1, 1])
        filt["index"] = fcols[0].number_input("Index Target", min_value=0, value=int(filt.get("index", 0)), key=f"fi_{i}")
        filt["op"] = fcols[1].selectbox("Operator", options=FILTER_OPS, index=FILTER_OPS.index(filt.get("op", "==")), key=f"fo_{i}")
        filt["value"] = fcols[2].number_input("Byte Constraint", min_value=0, max_value=255, value=int(filt.get("value", 0)), key=f"fv_{i}")
        if fcols[3].button("🗑", key=f"rm_filter_{i}", use_container_width=True):
            remove_filter_idxs.append(i)
        else:
            parsed_filters.append(Filter(index=int(filt["index"]), op=str(filt["op"]), value=int(filt["value"])))

    if remove_filter_idxs:
        st.session_state.receive_filters = [f for j, f in enumerate(filter_rows) if j not in remove_filter_idxs]
        st.rerun()

    bridge.set_subscriptions(selected_cmds, parsed_filters)

    # Hand over to the isolated fragment rendering module
    _render_isolated_log_terminal(bridge, selected_cmds, refresh_ms)


def main() -> None:
    st.set_page_config(page_title="Master Control Station", page_icon="📡", layout="wide")
    st.title("📡 Master System Communication Control Panel")

    _init_session_state()
    bridge = _get_bridge()
    refresh_ms = _render_sidebar(bridge)

    send_tab, receive_tab = st.tabs(["📤 Command Uplink Hub", "📥 Diagnostic Telemetry Logs"])
    with send_tab:
        _render_send_tab(bridge)
    with receive_tab:
        _render_receive_tab(bridge, refresh_ms)


if __name__ == "__main__":
    main()
