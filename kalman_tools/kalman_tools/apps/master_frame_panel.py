import os
import streamlit as st
import roslibpy

st.set_page_config(page_title="Master Sender", layout="centered")
st.title("📡 Master Frame Sender")

ROSBRIDGE_HOST = os.environ.get("KALMAN_TOOLS_ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.environ.get("KALMAN_TOOLS_ROSBRIDGE_PORT", "3001"))

if "ros_client" not in st.session_state:
    st.session_state.ros_client = None

if st.session_state.ros_client is None or not st.session_state.ros_client.is_connected:
    try:
        client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
        client.run(timeout=1.5)
        if client.is_connected:
            st.session_state.ros_client = client
    except Exception:
        st.session_state.ros_client = None
client = st.session_state.ros_client

if client and client.is_connected:
    st.success(f"Connected to Rosbridge at ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")

    if st.button("Send Message to Master", type="primary", use_container_width=True):
        try:
            talker = roslibpy.Topic(client, '/master_com/ros_to_master', 'kalman_interfaces/msg/MasterMessage')

            payload = {'data': [100, 1, 0, 255]}

            talker.publish(roslibpy.Message(payload))
            st.toast("Message successfully sent to master!", icon="🚀")

            talker.unadvertise()
        except Exception as e:
            st.error(f"Failed to transmit frame: {e}")
else:
    st.error(f"Offline: Unable to reach Rosbridge server at ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
    if st.button("Retry Connection"):
        st.rerun()
