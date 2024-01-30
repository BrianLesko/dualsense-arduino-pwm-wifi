##################################################################
# Brian Lesko
# 12/24/2023
# Robotics Studies, control a Stepper motor with a DualSense controller

import streamlit as st
import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt
import time
import math

import dualsense # DualSense controller communication
import customize_gui # streamlit GUI modifications
import robot
from ethernet import ethernet as eth # This class contains the methods 
DualSense = dualsense.DualSense # this class contains methods to communicate with the DualSense controller
gui = customize_gui.gui() # this class contains methods to modify the streamlit GUI
my_robot = robot.one_rev_joint() # This class contains the methods necessary to generate the robot figure

def main():
    # Set up the app UI
    gui.clean_format(wide=True)
    gui.about(text = "This code implements control of a simple robot with a ps5 remote.")
    Title = st.empty()
    subTitle = st.empty()
    with st.sidebar: Sidebar = st.empty()
    image_spot = st.empty()
    with st.sidebar: message_spot = st.empty()
    
    # Setting up the dualsense controller connection
    with message_spot: st.write("DualSense Controller Connection")
    vendorID, productID = int("0x054C", 16), int("0x0CE6", 16)
    ds = DualSense(vendorID, productID)
    try: ds.connect()
    except Exception as e:
        st.error("Error occurred while connecting to Dualsense controller. Make sure the controller is wired up and the vendor and product ID's are correctly set in the python script.")
    
    # Initialize loop variables
    with message_spot: st.write("Init Loop Vars")
    step = .001
    theta = 0.0
    prev_L1, prev_R1 = False, False
    th = robot.CyclicVariable(theta)

    # Control Loop
    with message_spot: st.write("Start Control Loop")
    while True:
        ds.receive()
        ds.updateTriggers()
        ds.updateThumbsticks()
        dtheta = 0.0

        # Button Control
        if ds.L2 > 0:
            dtheta = step*ds.L2/1.5
        if ds.R2 > 0:
            dtheta = - step*ds.R2/1.5
        with Sidebar: 
            st.write(f"Current Angle: {theta} \n \n")

        # Thumbstick control
        ds.updateThumbsticks()
        if ds.RX > 5 or ds.RX < -5:
            with Sidebar: 
                st.write(f"Current Angle: {theta} \n \n Desired Angle: {math.degrees(ds.Rthumb)}") # + "Current Angle: " + thetas[i]
            distance = ds.Rthumb - theta
            # Adjust distance to be between -pi and pi
            while distance < -math.pi: distance += 2 * math.pi
            while distance > math.pi:  distance -= 2 * math.pi
            dtheta = step*333*(distance)
        
        theta = theta + dtheta

        stepsize = 360/(200*4) # degrees per step 

        steps = round(math.degrees(dtheta) / stepsize,2)
        with message_spot: st.write(f"Steps: {steps}")
        # Send the command via UDP/IP to the robot
        if 'client' not in st.session_state:
             st.session_state.client = eth("client",'192.168.1.75', 12345)
        #    st.session_state.client.connect()
        
        bytes = str(steps).encode()
        st.session_state.client.s.sendto(bytes, ('192.168.1.75', 12345))

        # Show the robot
        fig = my_robot.get_robot_figure(theta)
        with image_spot: st.pyplot(fig)
        plt.close(fig)

main()