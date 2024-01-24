# Px4_Tilting_Custom_Control
 Customization of PX4 firmware to introduce a custom flight control stack 

## Efficient Development of Model-Based Controllers in PX4 Firmware: A Template-Based Customization Approach
__abstract__ This repo presents an approach to customize the PX4 firmware, focusing on implementing a modular template for the efficient development of new model-based controllers alongside the existing control stack. The proposed template provides a clear and flexible structure that helps developers to easily integrate new control algorithms, leveraging all the existing firmware functionalities. This customization methodology has been tested on various aerial platforms, demonstrating its effectiveness in improving performance and enabling the rapid adoption of new controllers. 
This work wants to enhance the field of flight firmware design, aiming to offer a solid foundation for developing customized and specialized solutions for a wide range of aerospace applications.
## Article
The description of the firmware architecture, the integration with the standard PX4 control stack, as well as its integration with a PixHawk autopilot, is described in the following article:

Simone D'Angelo, Francesca Pagano, Francesco Longobardi, Fabio Ruggiero, Vincenzo Lippiello, "Efficient Development of Model-Based Controllers in PX4 Firmware: A Template-Based Customization Approach", submitted to the 2024 International Conference on Unmanned Aircraft System (ICUAS ’24)  June 4-7, Chania, Crete, Greece

This work is currently under review

## Video


# How to use
Clone the repository with submodules <br />
`git clone --recurse-submodule https://github.com/prisma-lab/PX4_tilting_multicopters.git`

## Run the simulation
For omnidirectional tilting drone <br />
`make px4_sitl gazebo_NDT_tilting`
 or <br />
`make px4_sitl gazebo_NDT_tilting_interaction`

For one-tilt tilting drone <br />
`make px4_sitl gazebo_baby_k`
or  <br />
`make px4_sitl gazebo_baby_k_interaction`

Trigger the custom flight modes:<br />
`commander mode prisma:man` to send command through RC<br />
or<br />
`commander mode prisma:prisma1` to send command through ROS2 network


