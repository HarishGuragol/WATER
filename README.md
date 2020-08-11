# WATER

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

### To be done
1. Add plugin for dynamic mass calculation.
2. Add the waste detection algorithm.
3. Add the path planning algorithm.
4. Modify the model according to our design.
5. <del> Generate demo worlds.</del>
6. <del> Make the boat mobile. (The boat doesn't actually move due to abence of water, but its mobile)</del>
