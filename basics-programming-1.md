# ROS - Beginning Programming (Note 3)

Create a catkin workspace, this is a file structure protocol recommended by ros. Various `catkin` tools will help maintain the workspace file structure, generate some level of boilerplate code for the build scripts, and will also build the projects in a proper way.

## Creating and Building a Catkin workspace

[Catkin Workspace tutorial on Ros wiki](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

This tutorial assumes that you have installed catkin and sourced your environment. If you installed catkin via apt-get for ROS Indigo, your command would look like this: 
```
    $ source /opt/ros/indigo/setup.bash
```
Let's create a catkin workspace: 

```
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
```

Even though the workspace is empty (there are no packages in the 'src' folder, just a single CMakeLists.txt link) you can still "build" the workspace: 

```
$ cd ~/catkin_ws/
$ catkin_make
```

The `catkin_make` command is a convenience tool for working with catkin workspaces. If you look in your current directory you should now have a 'build' and 'devel' folder. Inside the 'devel' folder you can see that there are now several setup.*sh files. Sourcing any of these files will overlay this workspace on top of your environment. To understand more about this see the general catkin documentation: catkin. Before continuing source your new setup.*sh file: 

```
$ source devel/setup.bash
```
Using `catkin_make` to build a catkin workspace is very simple. You must call it in the top level of your catkin workspace. A typical workflow is: 

```
$ cd ~/catkin_ws/src/beginner_tutorials/src


# Add/Edit source files 

$ cd ~/catkin_ws/src/beginner_tutorials

# Update CMakeFiles.txt to reflect any changes to your sources 

$ cd ~/catkin_ws

$ catkin_make -DCMAKE_BUILD_TYPE=Release

```

Note: You might want to select a different CMake build type (e.g. RelWithDebInfo or Debug, see http://cmake.org/cmake/help/v2.8.12/cmake.html#variable:CMAKE_BUILD_TYPE).

This will build any packages in the source space (~/catkin_ws/src) to the build space (~/catkin_ws/build). Any source files, python libraries, scripts or any other static files will remain in the source space. However, any generated files such as libraries, executables, or generated code will be placed in the devel space. Also in the devel space there will be setup.*sh files generated, which when sourced will prefix your environment with this devel space. 

If you want you can pass any arguments you would normally pass to make to catkin_make. For instance, you can tell it to make the install target: 

```
$ cd ~/catkin_ws
$ catkin_make install
```

This will be equivalent to calling `cd ~/catkin_ws/build && make install`. Now there should be an install space (~/catkin_ws/install), which contains its own setup.*sh files. Sourcing one of these setup.*sh files will overlay this install space onto your environment. 

Note you should use either the *install* space or the *devel* space, not both at the same time. The devel space is useful when you are developing on the packages in your workspace, because then you don't have to invoke the install target each time, which is particularly useful when developing on Python or when running tests. 

The **install** space is useful when you are ready to distribute or 'install' the packages in your workspace. For instance, when building ROS from source you can use the 'install' target to install it to the system by passing the CMake argument '-DCMAKE_INSTALL_PREFIX=/opt/ros/groovy' to catkin_make: 

```
# This is an example
$ cd ~/catkin_ws
$ catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/groovy  # might need sudo
```

If you have a previously compiled workspace and you add a new package inside it, you can tell catkin to add this new package to the already-compiled binaries by adding this parameter: 
```
$ catkin_make --force-cmake
```

If you want to see the command lines used to run the compiler and linker, run catkin_make with this option: 
```
$ catkin_make -DCMAKE_VERBOSE_MAKEFILE=ON
```
Feel free to read more details about the inner workings of catkin_make or continue on to the next tutorial: Overlaying catkin packages using workspaces. 

## Creating a ROS Package

### Packages in a catkin Workspace
The recommended method of working with catkin packages is using a catkin workspace, but you can also build catkin packages standalone. A trivial workspace might look like this: 
```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```

Creating a catkin Package
This tutorial will demonstrate how to use the `catkin_create_pkg` script to create a new catkin package, and what you can do with it after it has been created. 

First change to the source space directory of the catkin workspace you created in the Creating a Workspace for catkin tutorial: 

```
# You should have created this in the Creating a Workspace Tutorial
$ cd ~/catkin_ws/src
```

Now use the `catkin_create_pkg` script to create a new package called `'beginner_tutorials'` which depends on `std_msgs`, `roscpp`, and `rospy`: 

```
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

This will create a beginner_tutorials folder which contains a `package.xml` and a `CMakeLists.txt`, which have been partially filled out with the information you gave `catkin_create_pkg`. 

`catkin_create_pkg` requires that you give it a package_name and optionally a list of dependencies on which that package depends: 

```
# This is an example, do not try to run this
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

catkin_create_pkg also has more advanced functionalities which are described in the catkin documentation. (/commands/catkin_create_pkg) 

### Building a catkin workspace and sourcing the setup file

Now you need to build the packages in the catkin workspace: 

```
$ cd ~/catkin_ws
$ catkin_make
```

After the workspace has been built it has created a similar structure in the devel subfolder as you usually find under /opt/ros/$ROSDISTRO_NAME. 

To add the workspace to your ROS environment you need to source the generated setup file: 

```
$ . ~/catkin_ws/devel/setup.bash
```
### Package dependencies

####First-order dependencies

When using catkin_create_pkg earlier, a few package dependencies were provided. These first-order dependencies can now be reviewed with the rospack tool. 

```
$ rospack depends1 beginner_tutorials 
std_msgs
rospy
roscpp
```

As you can see, rospack lists the same dependencies that were used as arguments when running catkin_create_pkg. These dependencies for a package are stored in the package.xml file: 

```
$ roscd beginner_tutorials
$ cat package.xml
<package>
...
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
...
</package>
```

#### Indirect dependencies

In many cases, a dependency will also have its own dependencies. For instance, rospy has other dependencies. 

```
$ rospack depends1 rospy
genpy
rosgraph
rosgraph_msgs
roslib
std_msgs
```

A package can have quite a few indirect dependencies. Luckily rospack can recursively determine all nested dependencies. 

```
$ rospack depends beginner_tutorials
cpp_common
rostime
roscpp_traits
roscpp_serialization
genmsg
genpy
message_runtime
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
catkin
rospack
roslib
rospy
```

### Customizing Your Package
This part of the tutorial will look at each file generated by catkin_create_pkg and describe, line by line, each component of those files and how you can customize them for your package. 

#### Customizing the package.xml

The generated package.xml should be in your new package. Now lets go through the new package.xml and touch up any elements that need your attention. 

*Description tag*

First update the description tag: 

```
  <description>The beginner_tutorials package</description>
```

Change the description to anything you like, but by convention the first sentence should be short while covering the scope of the package. If it is hard to describe the package in a single sentence then it might need to be broken up. 

*Maintainer tags*

Next comes the maintainer tag: 

```
  <maintainer email="user@todo.todo">user</maintainer>
```

This is a required and important tag for the package.xml because it lets others know who to contact about the package. At least one maintainer is required, but you can have many if you like. The name of the maintainer goes into the body of the tag, but there is also an email attribute that should be filled out: 


```
  <maintainer email="you@yourdomain.tld">Your Name</maintainer>
```

*license tags*

Next is the license tag, which is also required: 

```
  <license>TODO</license>
```

You should choose a license and fill it in here. Some common open source licenses are BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, and LGPLv3. You can read about several of these at the Open Source Initiative. For this tutorial we'll use the BSD license because the rest of the core ROS components use it already: 

```
  <license>BSD</license>
```

*Dependencies tags*

The next set of tags describe the dependencies of your package. The dependencies are split into build_depend, buildtool_depend, run_depend, test_depend. For a more detailed explanation of these tags see the documentation about Catkin Dependencies. Since we passed std_msgs, roscpp, and rospy as arguments to catkin_create_pkg, the dependencies will look like this: 

```
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
```

All of our listed dependencies have been added as a build_depend for us, in addition to the default buildtool_depend on catkin. In this case we want all of our specified dependencies to be available at build and run time, so we'll add a run_depend tag for each of them as well: 

```
  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>
```

*Final package.xml*

As you can see the final package.xml, without comments and unused tags, is much more concise: 

```
<?xml version="1.0"?>
<package>
  <name>beginner_tutorials</name>
  <version>0.1.0</version>
  <description>The beginner_tutorials package</description>

  <maintainer email="you@yourdomain.tld">Your Name</maintainer>
  <license>BSD</license>
  <url type="website">http://wiki.ros.org/beginner_tutorials</url>
  <author email="you@yourdomain.tld">Jane Doe</author>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>

</package>
```
