This is our new code that we will use in  European WRO finals in Croatia. We have used ROS which acts like a nervous system, the raspberrypi is the brain and the sensors are like the 6 senses. They help the robot understand where it is and what is around it. We did not want to flash our raspberry pi to ubuntu so we use a docker to bypass this issue. However we later learnt that ubuntu does not support our picam3. 

Camera publisher takes the values from the picam3(camera) and puts it through a bunch of python libraries, picam2(which gives us camera functionality) and OpenCV (which does the colour recognition). Then the code checks if the value matches specific rgb (red, green, blue values) that we have adjusted for green. Then it publishes it if it is green, to indicate where the green is, and if it is red, indicates the location of the red.

Camera receiver Node.py takes the values from the camera publisher and publishes it to the ROS system, for the navigation node to accept and use.

The Front TF Luna Node.py takes the values from the tf luna/ lidar which is connected via serial communication. This detects and converts it into floats, and broadcasts the information to navigation node.

Rear TF Luna Node.py takes the values from the tf luna/lidar in the rear which is connected on i2c and detects and  converts it into floats, in the same way as the front tf luna node, and broadcasts it to the navigation node.

IMU node takes the values from the gyro BNO085 and publishes it. However, our gyro has a nature of giving invalid information to the node, and prints error messages, which causes the gyro to crash which can be fatal to our code. So what we have done is when the code detects the gyro has crashed, it tries to restart the gyro. To prevent blind turning or blind driving we stop the robot.


Navigation node subscribes to all the ROS nodes (camera, receiver, Rear TF luna, IMU and Front TF luna node) taking the information that all the sensors see and completes the missions required (no camera publisher as our camera runs outside ROS as we could not get the camera libraries installed on the ROS/docker). Although this could create latency it has not noticeable done that yet. Our navigation node completes both open and obstacle and we can set up the different mode depending on what we want to run. 

The code starts by initializing all the variables (such as mode to decide which round we are completing).
Then it subscribes/ takes the broadcasted information ready to use to complete the missions.
Then we create the functions like Colour_detection which takes the information from the camera receiver node and decides if it should turn right or turn left.
Then the code calls the navigation loop which runs all the functions to complete everything required.
Then we have our "main" at the bottom which runs the navigation node and if we were to turn it off (using keyboard interrupt which means if we click a key it will disable the code for us) it makes sure it turns off the navigation node.


## Final Software Architecture

ROS 2 acts as the communication layer between our sensor and navigation nodes. The Raspberry Pi 4B acts as the main processing computer.

The camera processing runs outside ROS because we could not reliably run the required Picamera2 libraries inside our ROS Docker environment. The Camera Receiver therefore acts as the interface between the camera system and ROS.

```mermaid
flowchart TD
    PI["Raspberry Pi 4b <br/>BRAIN"]

    subgraph DOCKER["Docker Container"]
        ROS["ROS 2"]

        FRONT["Front TF-Luna Node"]
        REAR["Rear TF-Luna Node"]
        IMU["BNO085 IMU Node"]
        CAMREC["Camera Receiver"]
        NAV["Navigation Node"]

        FRONT --> NAV
        REAR --> NAV
        IMU --> NAV
        CAMREC --> NAV
    end

    subgraph CAMERA["Camera Processing<br/>(Outside ROS)"]
        PICAM["Picamera3"]
        CV["OpenCV"]
        COLOUR["Colour Detection<br/>Red / Green"]
        CAMPUB["Camera Publisher"]

        PICAM --> CV
        CV --> COLOUR
        COLOUR --> CAMPUB
    end

    PI --> DOCKER
    CAMPUB --> CAMREC
```
