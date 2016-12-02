

### About the ROS Solidworks URDF exporter

#
The Solidworks URDF exporter plugin allows you to convert a CAD model to a URDF file which can be used in various ROS packages.
Although the tool works, it doesnt work all the time. While generating the URDF for our Robot at [Octoloop](http://octoloop.in), we ran into many problems initially, since our model included a lot of complicated assemblies.

In this guide, we list out the guidelines and major steps in the process to run the tool reliably.

Here's the description from the [official wiki][wiki] of the plugin.
```
The SolidWorks to URDF exporter is a SolidWorks add-in that allows for the convenient export of SW Parts and Assemblies into a URDF file. 
The exporter will create a ROS-like package that contains a directory for meshes, textures and robots (urdf files). 
For single SolidWorks parts, the part exporter, will pull the material properties and create a single link in the URDF. 
For assemblies, the exporter will build the links, and createa tree based on the SW assembly hierarchy. 
The exporter can automatically determine the proper joint type, joint transforms and axes.
```

### Preparing your CAD model

#### Installing and setting up the plugin
1. __Use Solidworks 2012 64bit__ - The developer, Stephen Brawner built it to work with this version and since the tool isn't actively maintained since, right now the only reliabe version is this one. Here's what the wiki says - 
`So far this add-in has been tested only on Windows 7 64bit with SolidWorks 2012 64bit. It currently does not install on 32bit machines`
2. Install the [plugin][plugin_dl] and the latest .Net framework

Here is the repository for this plugin with the source code and the compiled .exe - https://bitbucket.org/brawner/sw2urdf

#### Preparing your CAD file

1. We built our CAD in Inventor and had a somewhat hard time exporting the URDF from the IPT/STEP files, at first.
2. The plugin normally allows you to select multiple parts as a link but it tends to freeze while eventually exporting.
3. With a lot of trial and error, we realised that the plugin works best and reliably when each **link is a single _part_ file**(SLDPRT file). 
4. So the final robotic arm assembly that you must export to URDF will have multiple links which are mated together at the joints.
Our links were complex assemblies with multiple parts per link, so we had to separately create a *single part file for each link assembly.* 
5. Also the **mating must only occur at one point for each joint**. 
6. Finally while exporting, **dont forget to mention the joint angle limits** since the URDF format now requires it. The plugin doesnt force you to put the limits, but later on ROS has a problem, especially when displaying the URDF via RViz.

### Using the Solidworks Plugin

1. The Tutorials for the plugin are pretty good.
2. http://wiki.ros.org/sw_urdf_exporter/Tutorials/Export%20a%20Part
3. http://wiki.ros.org/sw_urdf_exporter/Tutorials/Export%20an%20Assembly
4. When you export the urdf, you have to select a folder name. Remember that this folder name will also be your ROS package name. And since this tool generates all paths according to the package name, it becomes pretty critical that this name is simple and easy to remember. We will use this package name when we display the URDF in ROS in the next section.

### How to use the generated files on ROS

1. Once you have a folder ready, we will need to copy it to your linux system which has ROS installed.
2. When in your linux machine, [create a ROS catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), if you havent already done so. Lets say your ROS workspace is called `ros_ws` and it is located in your home(`~`) directory.
3. Now we will paste the generated urdf folder in the `/src` folder of the workspace.
4. Lets say your urdf folder is called `my_urdf`. Now as already mentioned earlier, your urdf's ROS package name is also `my_urdf`. 
5. We need the URDF files to be a ROS package for the original references to work properly.
6. For your urdf to be a ROS package, we need to generate some CMake and other files. Lets start -
   -  `cd` into your `ros_ws/src folder`
   
      ```
      cd ~/ros_ws/src
      ```
   - Now we will generate the necessary files for your urdf ros package by using `catkin_create_pkg`. While in the `src` directory itself, run the following command. Ensure you change `my_urdf` to your own urdf package name (it has to be the same as the urdf folder name you exported earlier)
   
     ```
     catkin_create_pkg my_urdf
     ```
   - Now all required files are generated. Lets compile the files now. `cd` to your workspace directory and run `catkin_make`
      
      ```
      cd ~/ros_ws
      catkin_make
      ```
   - Catkin should successfully compile your package, now your URDF is ready to be used in any ROS package/program
 7. Lets display the URDF in RViz along with a GUI so that you can change the joint angles.
 	- Ensure you are in your workspace directory. First lets source the setup.bash file from your `devel` folder. (remember to change the `~/ros_ws` path to your own ros workspace path)
 	  
      ```
      source ~/ros_ws/devel/setup.bash
      ```
 	- Now `cd` to the /src folder
 	- The generated URDF file is located in the `my_urdf/robots/` folder
 	- To display the file run the following commands
 		
        ```
        roslaunch urdf_tutorial display.launch model:=./my_urdf/robots/my_urdf.URDF gui:=True 
    	```
        Remember to install the `urdf_tutorials` package and to change `my_urdf` in the above paths to your own URDF package name.
    - Now RViz should open up with a window showing your robot and another window showing sliders to manipulate the joint angles

[wiki]: http://wiki.ros.org/sw_urdf_exporter
[plugin_dl]: https://bitbucket.org/brawner/sw2urdf/raw/27e55e0a7a64015f3c6d707cf38fe9a6975ef1c0/INSTALL/Output/sw2urdfSetup.exe

