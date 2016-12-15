# ObjectRecognition_Robot

1. This repository contains ROS Package for recognizing objects.<br> 
2. Package p3dx_description: Provide Pioneer model file with kinect camera for simulating and navigating.<br>
           p3dx_gazebo: launch file for spawning the robot<br>
           logical_camera: Send recognized object topic in gazebo to Ros<br> 
   Makingworldmap.m: Spawn the house model and objects.<br>

   When user press the key i(up),j(left),k(down),l(right), user can moving the robot. <br>
   A Robot recognize the object with kinect except for funiture models.<br>
   Visualize the recongized object models list in matlab.<br>

3. I will make robot move automatically. When user set the point, Robot will move to set point without key input.<br>
