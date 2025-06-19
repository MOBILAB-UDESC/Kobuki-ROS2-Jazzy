# Tools for graphics and saving data

import numpy as np
import os
import matplotlib.pyplot as plt

def save_data(xref, yref, thref, vref, wref, 
              x, y , theta, v , w,
              route):
    """
    Save results and reference
    
    Args:
        x, y, xref, yref (array): Position
        theta, thref (array)    : Orientation
        v, vref (array)         : Linear velocity input
        w, wref (array)         : Angular velocity input
        route (str)             : Path to save data
    """
    if not os.path.exists(route):
        os.makedirs(route)

    np.savetxt(route+"/Xref", xref, delimiter=',')
    np.savetxt(route+"/Yref", yref, delimiter=',')
    np.savetxt(route+"/Tref", thref, delimiter=',')
    np.savetxt(route+"/Vref", vref, delimiter=',')
    np.savetxt(route+"/Wref", wref, delimiter=',')
    np.savetxt(route+"/X", x, delimiter=',')
    np.savetxt(route+"/Y", y, delimiter=',')
    np.savetxt(route+"/Theta", theta, delimiter=',')
    np.savetxt(route+"/Vel", v, delimiter=',')
    np.savetxt(route+"/Wang", w, delimiter=',')

def draw_data(xref, yref, thref, vref, wref, 
              x, y , theta, v , w,
              route, name = ''):
    """
    Plot results vs reference
    
    Args:
        x, y, xref, yref (array): Position
        theta, thref (array)    : Orientation
        v, vref (array)         : Linear velocity input
        w, wref (array)         : Angular velocity input
        route (str)             : Path to save the image
        name (str)              : Name of the image
    """
    
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))
            
    # Plot 1: Trajectory
    axs[0, 0].plot(xref, yref, label="Referencia", linestyle="--")
    axs[0, 0].plot(x, y, label="Trajectory")
    axs[0, 0].set_xlabel("x")
    axs[0, 0].set_ylabel("y")
    axs[0, 0].set_title("Path tracking with LQR")
    axs[0, 0].legend()
    axs[0, 0].grid()
    axs[0, 0].axis("equal")
    
    # Plot 2: Linear Velocity
    axs[0, 1].plot(vref, label="Desired Linear Velocity")
    axs[0, 1].plot(v, label="Actual Linear Velocity")
    axs[0, 1].set_xlabel("Index")
    axs[0, 1].set_ylabel("Linear Velocity (m/s)")
    axs[0, 1].set_title("Linear Velocity over trajectory")
    axs[0, 1].legend()
    axs[0, 1].grid()
    
    # Plot 3: Angular Velocity
    axs[1, 0].plot(wref, label="Desired Angular Velocity")
    axs[1, 0].plot(w, label="Actual Angular Velocity")
    axs[1, 0].set_xlabel("Index")
    axs[1, 0].set_ylabel("Angular Velocity (rads/s)")
    axs[1, 0].set_title("Angular Velocity over trajectory")
    axs[1, 0].legend()
    axs[1, 0].grid()
    
    # Plot 4: Orientation
    axs[1, 1].plot(thref, label="Desired Orientation")
    axs[1, 1].plot(theta, label="Actual Orientation")
    axs[1, 1].set_xlabel("Index")
    axs[1, 1].set_ylabel("Orientation (rads)")
    axs[1, 1].set_title("Orientation over trajectory")
    axs[1, 1].legend()
    axs[1, 1].grid()
    
    plt.tight_layout()
    plt.savefig(route+"/"+name+".jpeg")
    plt.show()

def get_MAE(xref, yref, thref,
            x, y , theta,
            route):
    """
    Calculate MAE (Mean Absolute Error) for each output
    
    Args:
        x, y, xref, yref (array): Position
        theta, thref (array)    : Orientation
        route (str)             : Path to save results
    """
    
    mae = [0, 0, 0]
    l   = len(xref)

    for i in range(l):
        x_error = abs(x[i] - xref[i]) / l
        y_error = abs(y[i] - yref[i]) / l
        th_error = abs(theta[i] - thref[i]) / l
        mae[0] += x_error
        mae[1] += y_error
        mae[2] += th_error
    
    np.savetxt(route+"/MAE", mae, delimiter=',')

    return mae

def get_IAU_IADU(v, w, dt, route):
    """
    Calculate IAU (Integral of Absolute Input) and 
    IADU (Integral of Absolute Derivative of Input)
    
    Args:
        v (array): Linear velocity input
        w (array): Angular velocity input
        dt (float): Time step
        route (str): Path to save results
    """

    # Calculate IAU for both inputs
    # iau_v = np.sum(np.abs(v)) * dt
    # iau_w = np.sum(np.abs(w)) * dt
    iau_v = np.trapz(np.abs(v), dx=dt)
    iau_w = np.trapz(np.abs(w), dx=dt)
    
    # Calculate input derivatives
    dv = np.diff(v)
    dw = np.diff(w)
    
    # Calculate IADU for both inputs
    iadu_v = np.sum(np.abs(dv))
    iadu_w = np.sum(np.abs(dw))
    
    # Combine metrics
    metrics = np.array([
        iau_v,  # IAU for linear velocity
        iau_w,  # IAU for angular velocity
        iadu_v, # IADU for linear velocity
        iadu_w  # IADU for angular velocity
    ])
    
    # Save metrics to file
    np.savetxt(route+"/IAU_IADU", metrics, delimiter=',')
    
    return metrics

#!/usr/bin/env python3

# def generate_box_model(name, pose, size):
#     """Generate SDF model for a box"""
#     return f"""
#     <model name='{name}'>
#       <pose>{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}</pose>
#       <link name='box_link'>
#         <inertial>
#           <inertia>
#             <ixx>0.16666</ixx>
#             <ixy>0</ixy>
#             <ixz>0</ixz>
#             <iyy>0.16666</iyy>
#             <iyz>0</iyz>
#             <izz>0.16666</izz>
#           </inertia>
#           <mass>1</mass>
#           <pose>0 0 0 0 0 0</pose>
#         </inertial>
#         <collision name='box_collision'>
#           <geometry>
#             <box>
#               <size>{size[0]} {size[1]} {size[2]}</size>
#             </box>
#           </geometry>
#           <surface>
#             <friction>
#               <ode/>
#             </friction>
#             <bounce/>
#             <contact/>
#           </surface>
#         </collision>
#         <visual name='box_visual'>
#           <geometry>
#             <box>
#               <size>{size[0]} {size[1]} {size[2]}</size>
#             </box>
#           </geometry>
#           <material>
#             <ambient>1 1 1 1</ambient>
#             <diffuse>1 1 1 1</diffuse>
#             <specular>1 1 1 1</specular>
#           </material>
#         </visual>
#         <pose>0 0 0 0 0 0</pose>
#         <enable_wind>false</enable_wind>
#       </link>
#       <static>true</static>
#       <self_collide>false</self_collide>
#     </model>"""

# def generate_world_sdf():
#     """Generate complete world SDF"""
    
#     # Define your boxes here: (name, pose, size)
#     boxes = [
#         ('box_0', (9.5, 0.0, 0.5, 0.0, 0.0, 0.0), (1, 20, 1)),
#         ('box_1', (-9.5, 0.0, 0.5, 0.0, 0.0, 0.0), (1, 20, 1)),
#         ('box_2', (0.0, 9.5, 0.5, 0.0, 0.0, 0.0), (18, 1, 1)),
#         ('box_3', (0.0, -9.5, 0.5, 0.0, 0.0, 0.0), (18, 1, 1)),
#         ('box_4', (-5.5, 7.5, 0.5, 0.0, 0.0, 0.0), (1, 3, 1)),
#         ('box_5', (-3.5, 5.5, 0.5, 0.0, 0.0, 0.0), (1, 7, 1)),
#         ('box_6', (-0.5, 6.0, 0.5, 0.0, 0.0, 0.0), (1, 2, 1)),
#         ('box_7', (2.5, 8.0, 0.5, 0.0, 0.0, 0.0), (1, 2, 1)),
#         ('box_8', (-6.5, 1.5, 0.5, 0.0, 0.0, 0.0), (1, 7, 1)),
#         ('box_9', (-1.5, -2.5, 0.5, 0.0, 0.0, 0.0), (1, 7, 1)),
#         ('box_10', (4.5, -2.5, 0.5, 0.0, 0.0, 0.0), (1, 5, 1)),
#         ('box_11', (7.5, -1.0, 0.5, 0.0, 0.0, 0.0), (1, 4, 1)),
#         ('box_12', (7.5, -7.5, 0.5, 0.0, 0.0, 0.0), (1, 3, 1)),
#         ('box_13', (3.0, 3.0, 0.5, 0.0, 0.0, 0.0), (2, 2, 1)),
#         ('box_14', (-5.0, -6.0, 0.5, 0.0, 0.0, 0.0), (2, 2, 1)),
#         ('box_15', (-8.5, -6.0, 0.5, 0.0, 0.0, 0.0), (1, 2, 1)),
#         ('box_16', (-1.5, -7.5, 0.5, 0.0, 0.0, 0.0), (1, 1, 1)),
#         ('box_17', (3.5, -8.0, 0.5, 0.0, 0.0, 0.0), (1, 2, 1)),
#         ('box_18', (4.0, 7.5, 0.5, 0.0, 0.0, 0.0), (1, 2, 1)),
#         ('box_19', (-7.5, 4.5, 0.5, 0.0, 0.0, 0.0), (1, 1, 1)),
#         ('box_20', (-5.5, 3.5, 0.5, 0.0, 0.0, 0.0), (1, 1, 1)),
#         ('box_21', (3.5, -4.5, 0.5, 0.0, 0.0, 0.0), (1, 1, 1)),
#         ('box_22', (8.5, 4.5, 0.5, 0.0, 0.0, 0.0), (1, 1, 1)),
#         ('box_23', (8.5, -4.5, 0.5, 0.0, 0.0, 0.0), (1, 1, 1)),
#         ('box_24', (7.0, 6.5, 0.5, 0.0, 0.0, 0.0), (2, 1, 1)),
#         ('box_25', (1.0, 4.5, 0.5, 0.0, 0.0, 0.0), (4, 1, 1)),
#         ('box_26', (7.0, 6.5, 0.5, 0.0, 0.0, 0.0), (4, 1, 1)),
#         ('box_27', (7.5, 1.5, 0.5, 0.0, 0.0, 0.0), (3, 1, 1)),
#         ('box_28', (3.0, -0.5, 0.5, 0.0, 0.0, 0.0), (2, 1, 1)),
#         ('box_29', (2.0, -7.5, 0.5, 0.0, 0.0, 0.0), (2, 1, 1)),
#         ('box_30', (-3.0, -3.5, 0.5, 0.0, 0.0, 0.0), (2, 1, 1)),
#         # ... hasta 20 cajas o las que necesites
#     ]
    
#     sdf_header = """<?xml version="1.0"?>
# <sdf version='1.10'>
#   <world name='playground'>
#     <physics name='1ms' type='ignored'>
#       <max_step_size>0.005</max_step_size>
#       <real_time_factor>1</real_time_factor>
#       <real_time_update_rate>1000</real_time_update_rate>
#     </physics>
#     <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
#     <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
#     <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
#     <plugin name='gz::sim::systems::Contact' filename='gz-sim-contact-system'/>
#     <gravity>0 0 -9.8000000000000007</gravity>
#     <magnetic_field>5.5644999999999998e-06 2.2875799999999999e-05 -4.2388400000000002e-05</magnetic_field>
#     <atmosphere type='adiabatic'/>
#     <scene>
#       <ambient>0.400000006 0.400000006 0.400000006 1</ambient>
#       <background>0.699999988 0.699999988 0.699999988 1</background>
#       <shadows>true</shadows>
#     </scene>
    
#     <model name='ground_plane'>
#       <static>true</static>
#       <link name='link'>
#         <collision name='collision'>
#           <geometry>
#             <plane>
#               <normal>0 0 1</normal>
#               <size>100 100</size>
#             </plane>
#           </geometry>
#           <surface>
#             <friction>
#               <ode/>
#             </friction>
#             <bounce/>
#             <contact/>
#           </surface>
#         </collision>
#         <visual name='visual'>
#           <geometry>
#             <plane>
#               <normal>0 0 1</normal>
#               <size>100 100</size>
#             </plane>
#           </geometry>
#           <material>
#             <ambient>0. 0. 0. 0.</ambient>
#             <diffuse>0. 0. 0. 0.</diffuse>
#             <specular>0. 0. 0. 0.</specular>
#           </material>
#         </visual>
#         <pose>0 0 0 0 0 0</pose>
#         <inertial>
#           <pose>0 0 0 0 0 0</pose>
#           <mass>1</mass>
#           <inertia>
#             <ixx>1</ixx>
#             <ixy>0</ixy>
#             <ixz>0</ixz>
#             <iyy>1</iyy>
#             <iyz>0</iyz>
#             <izz>1</izz>
#           </inertia>
#         </inertial>
#         <enable_wind>false</enable_wind>
#       </link>
#       <pose>0 0 0 0 0 0</pose>
#       <self_collide>false</self_collide>
#     </model>"""

#     sdf_footer = """
#     <light name='sun' type='directional'>
#       <pose>0 0 10 0 0 0</pose>
#       <cast_shadows>true</cast_shadows>
#       <intensity>1</intensity>
#       <direction>-0.5 0.10000000000000001 -0.90000000000000002</direction>
#       <diffuse>0.800000012 0.800000012 0.800000012 1</diffuse>
#       <specular>0.200000003 0.200000003 0.200000003 1</specular>
#       <attenuation>
#         <range>1000</range>
#         <linear>0.01</linear>
#         <constant>0.90000000000000002</constant>
#         <quadratic>0.001</quadratic>
#       </attenuation>
#       <spot>
#         <inner_angle>0</inner_angle>
#         <outer_angle>0</outer_angle>
#         <falloff>0</falloff>
#       </spot>
#     </light>

#   </world>
# </sdf>"""

#     # Generate all boxes
#     boxes_sdf = ""
#     for name, pose, size in boxes:
#         boxes_sdf += generate_box_model(name, pose, size)
    
#     # Combine everything
#     complete_sdf = sdf_header + boxes_sdf + sdf_footer
    
#     return complete_sdf

# if __name__ == "__main__":
#     sdf_content = generate_world_sdf()
    
#     # Save to file
#     with open("/home/nilton/Desktop/Ros2/Kobuki/src/kobuki_ros/kobuki_description/worlds/playground.sdf", "w") as f:
#         f.write(sdf_content)
    
#     print("SDF file generated: generated_world.sdf")
#     print(f"Total lines: {len(sdf_content.split())}")