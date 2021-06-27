# WATER [Water Animosity Tainting and Emulsion Remover]

#### If you make any updates update the package.xml with verion number accordingly, and also mention the update in the readme file at the end.

To start simulation in Gazebo
> roslaunch WATER_description gazebo.launch

To view the model in Rviz
> roslaunch WATER_description display.launch

To start the Teleop Keyboard
> rosrun WATER_description propulsion_controller.py

To see the camera images from the boat
> roalaunch WATER_description gazebo.launch

> rosrun WATER_description image_capture

### Updates Made
1. Basic Design
2. Changed the Propllers
3. Boat is moving with Teleop keyboard
4. Added Water to the world
5. Added Object Detection Model

### To be done
1. Add plugin for dynamic mass calculation.
2. Add the path planning algorithm.
3. Modify the model according to our design.
4. <del>Add the waste detection algorithm</del>
5. <del> Generate demo worlds.</del>
6. <del> Make the boat mobile. (The boat doesn't actually move due to abence of water, but its mobile)</del>


### Budget
1) Robot Chassie - 10,000
	a) Body - 
	b) Motor Mounts
	c) Convyor Support
	d) Rudder
	e) Convyor Belt 
	f) Collecting Basket
	g) Propellers

2) Electronics - 
	a) SBC (Jetson  Nano ) - 5000
	b) Kinect Camera - 2000
	c) Motor Drivers - 1000
	d) Encoders - 200
	e) Wiring Kit - 500
	d) Misc Sensors - 500
 
3) Motors -
	a) Driver - 2000
	b) Stepper - 1200
	c) Servo  - 500

4) Power Supply -
	a) Batteries - 3000
	b) Solar Panels  - 2000
  
5)  Chemicals (Recurring) - 20,000
	a) Silver Nitrate Microparticles (Rod)
	b) Chlorides and Phosphate 
	c) Graphene
	d) 

Our Cost - 69,000
Avaerage Cost of Current Model - 12,000$ (Without Chemical Treatment)
