import os
import streamlit as st
from kalman_tools.ros_bridge import MasterRosBridge

ROSBRIDGE_HOST = os.environ.get("KALMAN_TOOLS_ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.environ.get("KALMAN_TOOLS_ROSBRIDGE_PORT", "3001"))

def get_ros_bridge() -> MasterRosBridge:
    """
    Retrieves or initializes the long-running MasterRosBridge instance
    inside Streamlit's session state.
    """
    if "ros_bridge" not in st.session_state:
        # MasterRosBridge handles background thread looping and auto-reconnection internally
        st.session_state.ros_bridge = MasterRosBridge(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    return st.session_state.ros_bridge

def send_custom_message(bridge: MasterRosBridge, cmd: int, data: list[int]) -> None:
    """
    Wraps the bridge's send_frame execution to keep message sending clean.
    """
    bridge.send_frame(cmd, data)

def page_panel(bridge: MasterRosBridge):
    st.set_page_config(page_title="Master Sender", layout="centered")
    st.title("📡 Master Frame Sender")

    # Utilize the bridge's thread-safe connection property
    if bridge.connected:
        st.success(f"Connected to Rosbridge at ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")

        if st.button("Send Message to Master", type="primary", use_container_width=True):
            try:
                # Retaining simple hardcoded values for testing
                send_custom_message(bridge, cmd=100, data=[0, 0])
                st.toast("Message successfully sent to master!", icon="🚀")
            except Exception as e:
                st.error(f"Failed to transmit frame: {e}")
    else:
        st.error(f"Offline: Unable to reach Rosbridge server at ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
        if st.button("Check/Retry Status"):
            st.rerun()

def main() -> None:
    bridge = get_ros_bridge()
    page_panel(bridge)

if __name__ == "__main__":
    main()
