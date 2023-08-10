## Elephant Robot

The Elephant robot consists of the following main components.
1. Stacking system
2. Loading system
3. Shooting system
4. Targetting system

### Stacking system
The stacking system consists of a stacking lift connected with a 2 rails and a timing belt attached to a stepper motor. 

## Loading system
The loading system consists of a loading lift connected with a 2 rails and a timing belt attached to a stepper motor.
A 3D printed part is attached that moves along the rails and pushes the rings from the stack to the shooting mechanism

## Shooting system
A linear rail is mounted with 2 traveller where one traveller pulls the loading string and the other traveller holds the base to carry the loaded ring. A moving trigger mounted on a linear actuator controls the release position of the trigger.

## Targetting system
A Kinect camera is used to get the depth image of the target and the rings. The depth image is processed to get the distance of the target and the rings. The distance of the target is used to control the shooting mechanism and the distance of the rings is used to control the loading mechanism. The processing is done using a Raspberry PI 3B board and it communicates using Serial communication to the ESP32 controller that controls the linear actuator.

### Targetting system workflow
-[x] Setting up the power circuit to power up the Kinect camera and the Raspberry PI 3B board
-[x] Setting up the Raspberry PI 3B board with the required libraries
-[ ] Coding the image processing algorithm to identify the poles and calculate the distance to those rings using the feed of the Kinect camera
-[ ] Setting up the serial communication between the Raspberry PI 3B board and the ESP32 controller
-[ ] Programming the ESP32 controller to control the linear actuator based on the distance calculated by the Raspberry PI 3B board
