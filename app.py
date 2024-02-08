##################################################################
# Brian Lesko
# 1/29/2024
# Connect to a dualsense controller and send a UDP Signal to a certain IP address

import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import dualsense # DualSense controller communication
import customize_gui # streamlit GUI modifications
from ethernet import ethernet as eth # This class contains the methods 
DualSense = dualsense.DualSense # this class contains methods to communicate with the DualSense controller
gui = customize_gui.gui() # this class contains methods to modify the streamlit GUI

def main():
    # Set up the app UI
    gui.clean_format(wide=True)
    gui.about(text = "This code implements...")
    st.title("Control Input")
    col1, col2, col3 = st.columns([1,4,1])
    with col2: image_spot = st.empty()
    
    # Setting up the dualsense controller connection
    vendorID, productID = int("0x054C", 16), int("0x0CE6", 16)
    ds = DualSense(vendorID, productID)
    try: ds.connect()
    except Exception as e:
        st.error("Error occurred while connecting to Dualsense controller. Make sure the controller is wired up and the vendor and product ID's are correctly set in the python script.")

    fig, ax = plt.subplots()
    data, = ax.plot([],[],'o')
    ax.xaxis.set_visible(False)
    # Hide all spines in one line
    for spine in ax.spines.values():
        spine.set_visible(False)

    # Control Loop
    history = []
    while True:
        ds.receive()
        ds.updateTriggers()
        ds.updateThumbsticks()

        # Button Control
        power = 0
        if abs(ds.L2) > 4:
            power = ds.L2
        if abs(ds.R2) > 4:
            power = ds.R2

        # Send the command via UDP/IP
        if 'client' not in st.session_state:
             st.session_state.client = eth("client",'192.168.1.75', 12345)
        bytes = str(power).encode()
        st.session_state.client.s.sendto(bytes, ('192.168.1.75', 12345))

        # Update a plot with the current Input signal, Power.
        with image_spot:
            data.set_data(0,power)
            history.append(power)
            ax.set_ylim([0, max(history)+1])
            st.pyplot(fig)

main()