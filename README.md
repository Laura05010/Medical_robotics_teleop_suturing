# Teleoperation of Franka for suturing

#### Final project for CSC496: Medical Robotics 2024

## Introduction

Our team (Laura and Puneet) established a Shared Control Robot System for the Franka to be able to make sutures. Based on our research for this project, the Da Vinci Robot is typically used for these kind of applications. Nevertheless, we were inclined to explore the feasibility of utilizing the Franka Emika Robot for this particular application. We believe that this assignment
will challenge us to make the teleoperation (learned from assignment 1) to be more precise and to use our knowledge (from assignment 2) of placing the robot in specific positions to make the task smoother.

## Methods

### Setting up the environment

To create the suturing pad, we used a suturing mesh secured with rubber bands onto a wooden box filled with Jenga blocks to stabilize it. By adding weight to the box with the Jenga blocks, we ensured that the robot wouldn't topple it over in case of a collision. For the needle, we used a plastic stand with a hole where floral foam was inserted to provide cushioning and stability. Plastic brackets were secured on the table surrounding the setup, along with duct tape, to enhance stability in case of robot collisions during implementation of the program and operation. The plastic stand serves as the reference point for needle positioning, while the wooden box acts as the suturing board.

### Needle

We opted for a 12.8cm curved sewing needle for the robot end-effector to facilitate seamless handling and smoother suturing motion. The needle is distinguished by color-coding: green marks the initial grasping point, purple signifies where the user should insert the needle, and red or purple indicates where the needle should be pulled out from the suturing board.
![Needle Mapping](/writeup_images/needle_map.png)

### End-effector

In selecting an end effector, our priority was to find grippers that could securely grasp the needle while maintaining stability. Through our exploration of different grippers, we discovered a suitable model in the "Robot Learning of Shifting Objects for Grasping in Cluttered Environments" repository. This repository offers a gripper model [(accessible here)](https://github.com/pantor/learning-shifting-for-grasping/blob/master/cad-models/gripper.stl) that effectively addresses our requirements.

Considering that the 3D printed material of the robot gripper is plastic, both the needle and this material exhibit a deficiency in friction which negatively affected the precision of our method. In our initial attempt, we tried to enhance the grippers' functionality by applying painter's tape to provide a cushioning effect. Unfortunately, this approach proved ineffective as the needle would frequently slip out of the grippers.

We then decided to experiment with cutting a sheet of silicone to match the grippers' shape. Although this method increased friction and effectively secured the needle in various positions, we faced challenges when the end effector initiated a twisting motion along one axis on the suturing board. This action caused the needle to slowly rotate around the end effector in a perpendicular axis instead of penetrating the board on the other end. To address this issue, we devised a solution involving a jar opener pad composed of rubber, which gave us more grip due to its textured surface. A
dditionally, we strategically placed a small magnet between one of the grippers and the rubber material to help us attract the metal of the needle. This combination of materials helped us ensure that the needle remained securely held throughout the positioning and suturing procedures.

## System

We configured the robot to initially navigate to the designated needle gripping position, enhancing the surgeon's workflow efficiency. After reaching the needle location, the surgeon can utilize the following keyboard inputs to grip the needle and maneuver the robot along the X, Y, and Z axes as follows:

| Key | Command                  |
| --- | ------------------------ |
| W   | Move robot to the left   |
| S   | Move robot to the right  |
| A   | Move robot toward you    |
| D   | Move robot away from you |
| Z   | Move robot upward        |
| X   | Move robot downward      |
| H   | Open the gripper         |
| G   | Close the gripper        |

Our system provides the surgeon with the flexibility to transition between different precision modes.
This allows the surgeon to fine-tune the robot's end effector to exact positions and prevent joint limits from being reached

| Key | Command                                                     |
| --- | ----------------------------------------------------------- |
| O   | The robot moves in general mode at a step distance of 0.11m |
| P   | The robot moves in precise mode at a step distance of 0.01m |

To navigate the robot to the suturing pad, the surgeon initiates the process by pressing the "K" key and then utilize precision modes to accurately position the needle onto the suturing pad. Once the surgeon is prepared to execute the stitch, they can simply press "M" to activate the end effector's twisting piercing motion.

| Key | Command                                                      |
| --- | ------------------------------------------------------------ |
| K   | Navigates robot to hover in the position of the suturing pad |
| M   | Activates end effector's piercing rotation                   |

After successfully piercing the skin, the surgeon can release the needle, maneuver to the opposite side using the precise mode keys, grasp the needle from the red/purple section, and complete the final loop by rotating the end effector.

**NOTE FOR FUTURE DEVELOPERS**

Note that the teleoperation commands for moving the robot along the x, y, and z axes update the (x, y, z) coordinates in 3D space by a specified step size. Inverse kinematics are then applied using the ik.controller to determine the new position of the robot. We opted for this approach in teleoperation as it offers intuitive control for a surgeon compared to manually moving each of the robot's joints with keyboard input to acheive a specific position.

On the other hand, the process for placing the robot in the position to grasp the needle and in the position of the suturing board, puts the robot in the specific position by specifying the values for each of the 7 joints of the Franka emika robot and then using the MotionGenerator, this method was inportant in order to avoid joint limits and collision of the robot with the objects in its vicinity. _Please make sure to adjust these values to match your needs based on the your positions for the needle and the suturing pad._

## Tasks and their distribution

During the course of the project, our team established several milestones to ensure progress towards our ultimate objective. The following key milestones played a crucial role in achieving the success of our project:

1. Establish a stable environment and conduct simulated collision tests to assess robustness.
2. Teleoperation Phase 1: Successfully maneuver the robot using designated keys to control movement in all directions: up, down, left, right, forward, and backward.
3. Teleoperation Phase 2: Develop two operational modes (general and precise) to adjust the step size of robot movement in specific directions.
4. Position the robot at the needle insertion point by fine-tuning joint values.
5. Position the robot at the suturing board location by adjusting joint values, and ensure collision avoidance with nearby objects.
6. Implement a keyboard command to twist the robot's 7th joint for performing skin puncturing and completing the suture loop procedure.

## How to run the program

### **1. TURNING ON THE ROBOT**

1. Pull the "red" colored power switch.
2. Wait for status LED to turn a solid yellow.
3. Open the chrome browser and type `192.168.1.107` to access Franka Desk.
4. Unlock the joints by clicking the unlock icon.
5. Go to the hamburger menu and click **Settings**
6. Click on the End-Effector tab
7. Click on the **Homing button** and wait for the robot's end-effector to initialize
8. Once the robot is done Homing, press the Blue **APPLY button**
9. Go to the hamburger menu and click **Desk**
10. Unlock the joints of the robot if they are locked
11. Go to the hamburger menu and click **Activate FCI**. Let it remain active.

### **2. STARTING THE DOCKER CONTAINER**

1. Open terminal in the computer
2. `cd /csc496`
3. run the command `./start_docker_container`
4. run the command `./open_docker_container`

Once the container has opened, you can start the program.

### **3. RUNNING THE PROGRAM**

1. `cd /csc496/runners/build`
2. run the command `cmake ..`
3. run the command `make`
4. run the following command `./run_final_proj 192.168.1.107`
   **Note that you can find the main program for this project on /csc496/runners/final_proj.cpp**

### **4. INTERACTING WITH THE PROGRAM**

Follow the instructions on the terminal but these are the general instructions:

1. Press `Enter` to put the robot in position for grabbing the needle
2. We the terminal says "Ready to move: teleop", you must use the keys to operate the robot, for instructions and meaning of the keys please go to the **System section** of this document

### **5. STOPING THE PROGRAM**

1. Hit `CTRL+C` and the program will stop

### **6. STOPING THE DOCKER CONTAINER**

1. type `exit`
2. run the command `./stop_docker_container`
3. close terminal

### **7. TURNING OFF THE ROBOT**

1. Open Franka Desk.
2. Deactivate FCI.
3. Lock the joints
4. Go to the hamburger menu and select `Shutdown Robot`.
5. Once Franka Desk says it is okay to power off, then press the red power switch.
