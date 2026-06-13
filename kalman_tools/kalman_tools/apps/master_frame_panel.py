import os
import streamlit as st
import roslibpy

st.set_page_config(page_title="Master Sender", layout="centered")
st.title("📡 Master Frame Sender")

# 1. Fetch connection details injected by your ROS 2 launch script
ROSBRIDGE_HOST = os.environ.get("KALMAN_TOOLS_ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.environ.get("KALMAN_TOOLS_ROSBRIDGE_PORT", "3001"))

# 2. Maintain a single, persistent connection instance across Streamlit reruns
if "ros_client" not in st.session_state:
    st.session_state.ros_client = None

# Establish connection if it doesn't exist or dropped
if st.session_state.ros_client is None or not st.session_state.ros_client.is_connected:
    try:
        client = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
        client.run(timeout=1.5)  # Spin connection loop in background thread
        if client.is_connected:
            st.session_state.ros_client = client
    except Exception:
        st.session_state.ros_client = None

# 3. Render Interface based on Connection Status
client = st.session_state.ros_client

if client and client.is_connected:
    st.success(f"Connected to Rosbridge at ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")

    # The single button requested to dispatch the frame
    if st.button("Send Message to Master", type="primary", use_container_width=True):
        try:
            # Instantiate the publisher channel mapping your target topic
            talker = roslibpy.Topic(client, '/master_com/ros_to_master', 'kalman_interfaces/msg/MasterMessage')

            # Sample payload structure matching an Int32MultiArray
            payload = {'data': [100, 1, 0, 255]}

            talker.publish(roslibpy.Message(payload))
            st.toast("Message successfully sent to master!", icon="🚀")

            # Unadvertise immediately since this is a single, on-demand event trigger
            talker.unadvertise()
        except Exception as e:
            st.error(f"Failed to transmit frame: {e}")
else:
    st.error(f"Offline: Unable to reach Rosbridge server at ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
    if st.button("Retry Connection"):
        st.rerun()
