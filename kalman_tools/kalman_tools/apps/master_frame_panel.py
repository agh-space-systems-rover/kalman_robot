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


# ── Singleton bridge ──────────────────────────────────────────────────────────

@st.cache_resource
def _get_bridge() -> MasterRosBridge:
    host = os.environ.get("KALMAN_TOOLS_ROSBRIDGE_HOST", "localhost")
    port = int(os.environ.get("KALMAN_TOOLS_ROSBRIDGE_PORT", "3001"))
    return MasterRosBridge(host=host, port=port)


# ── Session state init ────────────────────────────────────────────────────────

def _init_session_state() -> None:
    if "send_rows" not in st.session_state:
        st.session_state.send_rows = [{**DEFAULT_SEND_ROW, "id": str(uuid.uuid4())}]
    if "receive_cmds" not in st.session_state:
        st.session_state.receive_cmds = []
    if "receive_filters" not in st.session_state:
        st.session_state.receive_filters = []


# ── Catalog helpers ───────────────────────────────────────────────────────────

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


# ── Sidebar ───────────────────────────────────────────────────────────────────

def _render_sidebar(bridge: MasterRosBridge) -> float:
    st.sidebar.header("Connection")
    st.sidebar.text_input("Host", value=bridge.host, disabled=True)
    st.sidebar.number_input(
        "Port",
        min_value=1,
        max_value=65535,
        value=int(bridge.port),
        disabled=True,
        help="Set via launch arg `rosbridge_port`. GS uses 9065.",
    )
    st.sidebar.caption("Restart app to change host/port.")
    if bridge.connected:
        st.sidebar.success("Connected")
    else:
        st.sidebar.warning("Waiting for rosbridge…")
        # Auto-poll until connected; stops as soon as bridge.connected is True.
        st.markdown(
            "<meta http-equiv='refresh' content='1'>",
            unsafe_allow_html=True,
        )

    refresh_ms = st.sidebar.number_input(
        "Receive auto-refresh (ms, 0 = off)",
        min_value=0,
        max_value=10000,
        value=0,
        step=100,
    )
    return float(refresh_ms)


# ── Send tab ──────────────────────────────────────────────────────────────────

def _render_send_tab(bridge: MasterRosBridge) -> None:
    st.subheader("Send frames")
    st.caption("Publishes to `master_com/ros_to_master`.")

    catalog_labels = _catalog_labels()
    rows: list[dict] = st.session_state.send_rows

    # ── add row ──
    if st.button("＋ Add frame row", key="add_send_row"):
        rows.append({**DEFAULT_SEND_ROW, "id": str(uuid.uuid4())})
        st.rerun()

    remove_ids: list[str] = []
    duplicate_rows: list[dict] = []

    for row in list(rows):
        row_id = row["id"]

        with st.container(border=True):

            # ── header row: label + remove/duplicate ──
            hcols = st.columns([4, 1, 1])
            with hcols[0]:
                row["name"] = st.text_input(
                    "Label (optional)",
                    key=f"send_name_{row_id}",
                )
            with hcols[1]:
                if st.button("🗑", key=f"remove_send_{row_id}", help="Remove row"):
                    remove_ids.append(row_id)
                    bridge.stop_spam(row_id)
            with hcols[2]:
                if st.button("⎘", key=f"dup_send_{row_id}", help="Duplicate row"):
                    duplicate_rows.append({**row, "id": str(uuid.uuid4())})

            # ── frame selection ──
            fcols = st.columns([3, 3])
            with fcols[0]:
                st.selectbox(
                    "Frame (catalog)",
                    options=catalog_labels,
                    index=_catalog_index(
                        st.session_state.get(f"send_cmd_{row_id}", row.get("cmd_text", "")),
                        catalog_labels,
                    ),
                    key=f"send_catalog_{row_id}",
                    on_change=_on_catalog_change,
                    args=(row_id, catalog_labels),
                    help="Pick from known frame IDs. You can also type manually below.",
                )
            with fcols[1]:
                cmd_text: str = st.text_input(
                    "Cmd (name / 0xNN / decimal)",
                    key=f"send_cmd_{row_id}",
                    help="e.g. AUTONOMY_SWITCH  or  0x20  or  32",
                )

            # ── payload + interval ──
            pcols = st.columns([3, 2])
            with pcols[0]:
                payload_text: str = st.text_input(
                    "Payload bytes",
                    key=f"send_payload_{row_id}",
                    help="Comma/space separated 0–255, e.g.  1, 0, 255",
                )
            with pcols[1]:
                interval_s: float = st.number_input(
                    "Spam interval (s)",
                    min_value=0.05,
                    value=float(row.get("interval_s", 1.0)),
                    step=0.05,
                    key=f"send_interval_{row_id}",
                )

            # ── parse & preview ──
            parse_error: str | None = None
            cmd: int | None = None
            data: list[int] | None = None
            try:
                cmd = parse_cmd(cmd_text)
                data = parse_payload(payload_text)
            except (ValueError, Exception) as exc:
                parse_error = str(exc)

            if parse_error:
                st.error(f"Parse error: {parse_error}")
            else:
                st.caption(
                    f"→ `master_com/ros_to_master`  cmd={format_cmd(cmd)}"  # type: ignore[arg-type]
                    f"  data=[{format_payload(data)}]"  # type: ignore[arg-type]
                )

            # ── action buttons ──
            acols = st.columns(3)
            disabled = parse_error is not None or not bridge.connected

            with acols[0]:
                if st.button(
                    "Send now",
                    key=f"send_now_{row_id}",
                    disabled=disabled,
                    type="primary",
                ):
                    try:
                        bridge.send_frame(cmd, data)  # type: ignore[arg-type]
                        st.toast(f"Sent cmd={format_cmd(cmd)} data=[{format_payload(data)}]", icon="🚀")
                    except RuntimeError as exc:
                        st.error(str(exc))

            with acols[1]:
                spamming = bridge.is_spamming(row_id)
                if not spamming:
                    if st.button(
                        "▶ Start spam",
                        key=f"start_spam_{row_id}",
                        disabled=disabled,
                    ):
                        try:
                            bridge.start_spam(row_id, cmd, data, interval_s)  # type: ignore[arg-type]
                            st.toast(f"Spamming every {interval_s} s", icon="📡")
                            st.rerun()
                        except (RuntimeError, ValueError) as exc:
                            st.error(str(exc))
                else:
                    if st.button(
                        "⏹ Stop spam",
                        key=f"stop_spam_{row_id}",
                        type="primary",
                    ):
                        bridge.stop_spam(row_id)
                        st.rerun()

            with acols[2]:
                if spamming:
                    st.markdown(
                        "<span style='color:#f0a500;font-weight:bold'>● spamming</span>",
                        unsafe_allow_html=True,
                    )

            # carry mutable fields back to session state dict
            row["interval_s"] = interval_s

    # apply removals / duplicates
    if remove_ids:
        st.session_state.send_rows = [r for r in rows if r["id"] not in remove_ids]
        st.rerun()
    if duplicate_rows:
        st.session_state.send_rows = rows + duplicate_rows
        st.rerun()

    st.divider()
    if st.button("⏹ Stop all spam", type="primary", key="stop_all_spam"):
        bridge.stop_all_spam()
        st.rerun()


# ── Receive tab ───────────────────────────────────────────────────────────────

def _render_receive_tab(bridge: MasterRosBridge, refresh_ms: float) -> None:
    st.subheader("Receive frames")
    st.caption("Subscribes to `master_com/master_to_ros/xNN`.")

    frames = list_frame_definitions()
    label_to_cmd = {frame.label: frame.cmd for frame in frames}

    selected_labels: list[str] = st.multiselect(
        "Frame IDs to display",
        options=[frame.label for frame in frames],
        default=st.session_state.receive_cmds,
        help="Multiple names can share the same cmd value (e.g. 0x53).",
    )
    st.session_state.receive_cmds = selected_labels
    selected_cmds = {label_to_cmd[lbl] for lbl in selected_labels}

    if selected_labels:
        seen: set[int] = set()
        lines = []
        for lbl in selected_labels:
            cmd = label_to_cmd[lbl]
            if cmd not in seen:
                seen.add(cmd)
                lines.append(f"`master_com/master_to_ros/{format_cmd(cmd)[1:]}`")
        st.caption("Listening on: " + "  ·  ".join(lines))

    # ── filters ──
    st.markdown("**Data filters** (all conditions must match — AND logic)")
    filter_rows: list[dict] = st.session_state.receive_filters
    if st.button("＋ Add filter", key="add_filter"):
        filter_rows.append({"index": 0, "op": "==", "value": 0})
        st.rerun()

    remove_filter_idxs: list[int] = []
    parsed_filters: list[Filter] = []
    for i, filt in enumerate(filter_rows):
        fcols = st.columns([1, 1, 1, 1])
        filt["index"] = fcols[0].number_input(
            "data[i]", min_value=0, value=int(filt.get("index", 0)), key=f"fi_{i}"
        )
        filt["op"] = fcols[1].selectbox(
            "op", options=FILTER_OPS,
            index=FILTER_OPS.index(filt.get("op", "==")),
            key=f"fo_{i}",
        )
        filt["value"] = fcols[2].number_input(
            "value", min_value=0, max_value=255,
            value=int(filt.get("value", 0)), key=f"fv_{i}",
        )
        if fcols[3].button("🗑", key=f"rm_filter_{i}"):
            remove_filter_idxs.append(i)
        else:
            parsed_filters.append(
                Filter(index=int(filt["index"]), op=str(filt["op"]), value=int(filt["value"]))
            )

    if remove_filter_idxs:
        st.session_state.receive_filters = [
            f for j, f in enumerate(filter_rows) if j not in remove_filter_idxs
        ]
        st.rerun()

    bridge.set_subscriptions(selected_cmds, parsed_filters)

    # ── log controls ──
    ctrl = st.columns(3)
    if ctrl[0].button("🗑 Clear log", key="clear_log"):
        bridge.clear_received_messages()
        st.rerun()
    if ctrl[1].button("⟳ Refresh", key="refresh_log"):
        st.rerun()

    # optional HTML auto-refresh (only when user explicitly enables it)
    if refresh_ms > 0:
        st.markdown(
            f"<meta http-equiv='refresh' content='{refresh_ms / 1000:.3f}'>",
            unsafe_allow_html=True,
        )

    messages = bridge.get_received_messages()

    if bridge.connected and selected_cmds and not messages:
        st.info("No messages yet. Ensure `master_com` or `master_loopback` is running.")

    if not messages:
        st.write("No received frames.")
        return

    rows_data = []
    for msg in reversed(messages):
        local_time = msg.time.astimezone().strftime("%H:%M:%S.%f")[:-3]
        rows_data.append(
            {
                "time": local_time,
                "cmd": _cmd_display(msg.cmd),
                "topic": msg.topic,
                "data (dec)": format_payload(msg.data),
                "data (hex)": format_payload_hex(msg.data),
            }
        )
    st.dataframe(rows_data, use_container_width=True, hide_index=True)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    st.set_page_config(
        page_title="Master Frame Panel",
        page_icon="📡",
        layout="wide",
    )
    st.title("📡 Master Frame Panel")
    st.caption("Send and inspect Master protocol frames via rosbridge.")

    _init_session_state()
    bridge = _get_bridge()
    refresh_ms = _render_sidebar(bridge)

    send_tab, receive_tab = st.tabs(["Send", "Receive"])
    with send_tab:
        _render_send_tab(bridge)
    with receive_tab:
        _render_receive_tab(bridge, refresh_ms)


if __name__ == "__main__":
    main()
