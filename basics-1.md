# ROS(Concepts) Basics - (Note 1)

## Basics

Every program in ROS is called a node. Nodes communicate with each other through channels. These channels 

Ros programs are built using a tool called `catkin`. It is built over `CMake` and works like `make`.

Every program in ROS needs a _Workspace_. 

```
A workspace is simply a set of directories in which a related set of ROS code lives.
You can have multiple ROS workspaces, but you can only work in one of them at any one time. 
The simple way to think about this is that you can only see code that lives in your current workspace
```

Workspaces further contain _packages_ which are somewhat like libraries for your program.

`rosrun` is used to run a single ros program(node), but many times, a robot system contains multiple nodes, hence `roslaunch` is used(discussed later)
 
`roscore` - Main program to run initiate ros. It sets up the basic architecture for the channels, allowing nodes to communicate. We need to run `roscore` everytime before starting any ROS node.

Since ROS programs(nodes) communicate with each other through predefined channels, it is difficult to visualise the current channels and the messages. Hence there are tools like `rostopic` and `rqt_graph` which make this visualisation easier.

Every channel can have any number of broadcasters and receivers, hence it is trivial to make simple _loggers_ for any ROS channel.

_Namespaces_ and _Remapping_ are used to handle nodes which are similar in features but different in function(eg two cameras or two arms in a robot)

Following the convention of Unix paths and Internet URIs, ROS uses the forward slash (/) to delimit namespaces. a robot with two cameras could launch two camera drivers in separate namespaces, such as _left_ and _right_ , which would result in image streams named _left/image_ and _right/image_.

Pushing a node into a namespace can be accomplished with a special `__ns` namespace-remapping syntax (note the double underscore). For example, if the working directory contains the camera program, the following shell command would launch camera into the namespace right:
`user@hostname$ ./camera __ns:=right`

_Remapping_ is used on the receiver end to listen to channels on different namespaces. In ROS, any string in a program that defines a name can be remapped at runtime.

For example, if the working directory contains the image_view program, one could type the following to map image to right/image:
```
user@hostname$ ./image_view image:=right/image
```

__Just as for filesystems, web URLs, and countless other domains, ROS names must be
unique. If the same node is launched twice, roscore directs the older node to exit to
make way for the newer instance of the node__

#### Roslaunch

`roslaunch` is used to automate launching multiple nodes at once files have a suffix of `.launch`

It also closes all nodes the `Ctrl-C` is pressed in the console where it was launched.

It automatically starts `roscore`, if not previously started.


#### The `tf` package

One problem that might not be immediately obvious, but is extremely important, is the management of coordinate frames. Seriously, coordinate frames are a big deal in robotics.

Let’s establish some terminology. In our 3D world, a position is a vector of three numbers (x, y, z) that describe how far we have translated along each axis, with respect to some origin. Similarly, an orientation is a vector of three numbers (roll, pitch, yaw)that describe how far we have rotated about each axis, again with respect to some origin. Taken together, a (position, orientation) pair is called a pose. For clarity, this kind of pose, which varies in six dimensions (three for translation plus three for rota‐
tion) is sometimes called a 6D pose. Given the pose of one thing relative to another, we can transform data between their frames of reference, a process that usually involves some matrix multiplications.

Any node can be the authority that publishes the current information for some transform(s), and any node can subscribe to transform data, gathering from all the various authorities a complete picture of the robot. This system is implemented in the tf (short for transform) package, which is extremely widely used throughout ROS software.

As is often the case for a powerful system, tf is relatively complex, and there are a variety of ways in which things can go wrong. Consequently, there a number of tf-specific introspection and debugging tools to help you understand what’s happening, from printing a single transform on the console to rendering a graphical view of the entire transform hierarchy. There is much, much more to know about the tf system, but for the work that we’ll do in the rest of this book(summary), this introduction should be enough for you to understand what’s happening. 

---

## Topics

Topics implement a publish/subscribe communication mechanism, one of the more common ways to exchange data in a distributed system. Before nodes start to transmit data over topics, they must first announce, or advertise, both the topic name and the types of messages that are going to be sent. Then they can start to send, or publish, the actual data on the topic. Nodes that want to receive messages on a topic can subscribe to that topic by making a request to roscore. After subscribing, all messages on the topic are delivered to the node that made the request. One of the main advantages to using ROS is that all the messy details of setting up the necessary connections when nodes advertise or subscribe to topics is handled for you by the underlying communication mechanism so that you don’t have to worry about it yourself

In ROS, all messages on the same topic must be of the same data type. Although ROS does not enforce it, topic names often describe the messages that are sent over them. For example, on the PR2 robot, the topic /wide_stereo/right/image_color is used for color images from the rightmost camera of the wide-angle stereo pair.

#### Publishing to a Topic

To send data on a topic, you have to be a publisher. It involves "announcing" that you will publish on a topic and mention the Data type that will be published.

Example in python

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

rospy.init_node('topic_publisher')

 # announcing that the topic name is "counter" and data type is Int32
pub = rospy.Publisher('counter', Int32)

rate = rospy.Rate(2) # defining the frequency of messages in Hz (msgs per sec)

count = 0
while not rospy.is_shutdown():
    pub.publish(count) # Sending the actual message
    count += 1
    rate.sleep() # uses previously set value to determine delay
```
Also, remember since topic names are unique, if you announce a topic who's name already exists, it get overwritten.(Check docs)

You can use `rostopic` to see the status. `rostopic list` shows all channels currently online. `rostopic echo topicname` will show the messages on the topic called _topicname_. Adding the `-n <num_msgs>` flag will limit the number of messages to `<num_msgs>`

#### Subscribing to a topic

For listening to a topic, ROS provides an asynchronous way to listen to the messages.

Example here,

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def callback(msg): # callback which is called everytime there is a message
    print msg.data

rospy.init_node('topic_subscriber')

# Send the subscribe info to roscore, if channel doesnt exist, it simply waits till a message comes on this channel
sub = rospy.Subscriber('counter', Int32, callback) 

rospy.spin() # give back control to ROS
```
#### Latched Topics

By default, you receive a message on a topic only when it is published. So if a subscriber subscribes on a channel after the message is published, it misses the message.

Latched topics ensure that every new subscriber receives the last broadcast message on the topic, when they subscribe.  This is how a topic is marked "latched"

`pub = rospy.Publisher('map', nav_msgs/OccupancyGrid, latched=True)`


#### Custom Message Types

ROS messages are defined by special message-definition files in the `msg` directory of a package. 
These files are then compiled into language-specific implementations that can be used in your code. So, even if you’re using an interpreted language such as Python, you need to run `catkin_make` if you’re going to define your own message types. Otherwise, the language-specific implementation will not be generated, and Python will not be able to find your new message type. Furthermore, if you don’t rerun catkin_make after you change the message definition, Python will still be using the older version of the message type.

Message-definition files are typically quite simple and short. Each line specifies a type and a field name. Types can be built-in ROS primitive types, message types from other packages, arrays of types (either primitive or from other packages, and either fixed or variable length), or the special _Header_ type

For example, 

Message definition file for a new type `Complex`

`Complex.msg`

```
float32 real
float32 imaginary
```
Once the message is defined, we need to run `catkin_make` to generate the language-specific code that will let us use it. This code includes a definition of the type, and code to marshal and unmarshal it for transmission down a topic. This allows us to use the message in all of the languages that ROS supports; nodes written in one language can subscribe to topics from nodes written in another. Moreover, it allows us to use messages to communicate seamlessly between computers with different architectures.

## Services

Services are another way to pass data between nodes in ROS. Services are just synchronous remote procedure calls; they allow one node to call a function that executes in another node.

Service calls are well suited to things that you only need to do occasionally and that take a bounded amount of time to complete.

The first step in creating a new service is to define the service call inputs and outputs. This is done in a _service-definition file_ , which has a similar structure to the message-definition files we’ve already seen. However, since a service call has both inputs and outputs, it’s a bit more complicated than a message.

Example
```
string words
---
uint32 count
```

The inputs to the service call come first. In this case, we’re just going to use the ROS built-in string type. Three dashes ( --- ) mark the end of the inputs and the start of
the output definition. We’re going to use a 32-bit unsigned integer ( uint32 ) for our
output. The file holding this definition is called WordCount.srv and is traditionally in
a directory called srv in the main package directory (although this is not strictly
required).

After saving this, run `catkin_make` to create the code and definitions. The `find_package()` call in _CMakeLists.txt_ needs to include `message_generation`.
Also another update required is the addition of `add_service_files()` section.

Also the package.xml needs to be updated as well(check book)
Refer Page 52 in the ROSBook

After configuring everything as mentioned in the book, `catkin_make` will generate the classes - _WordCount_, _WordCountRequest_ and _WordCountResponse_

Check the book for an explaination on services.


# Actions

ROS actions are the best way to implement interfaces to time-extended, goal-oriented behaviors like `goto_position` . 

While services are synchronous, actions are asynchronous. Similar to the request and response of a service, an action uses a goal to initiate a behavior and sends a result when the behavior is complete. But the action further uses feedback to provide updates on the behavior’s progress toward the goal and also allows for goals to be canceled. 

Actions are themselves implemented using topics. An action is essentially a higher-level protocol that specifies how a set of topics (goal, result, feedback, etc.) should be used in combination.

*For example* - Using an action interface to `goto_position` , you send a goal, then move on to other tasks while the robot is driving. Along the way, you receive periodic progress updates (distance traveled, estimated time to goal, etc.), culminating in a result message (did the robot make it to the goal or was it forced to give up?). And if something more important comes up, you can at any time cancel the goal and send the robot somewhere else. 

Actions require only a little more effort to define and use than do services, and they provide a lot more power and flexibility.

The first step in creating a new action is to define the goal, result, and feedback message formats in an action definition file, which by convention has the suffix _.action_. The _.action_ file format is similar to the _.srv_ format used to define services, just with an additional field. And, as with services, each field within an _.action_ file will become its own message.

Just like with service-definition files, we use three dashes ( --- ) as the separator between the parts of the definition. While service definitions have two parts (request and response), action definitions have three parts (goal, result, and feedback).

_This is an action definition file for simple Timer , which has three parts: the goal, the result, and the feedback._
```python

# Part 1: the goal, to be sent by the client

# The amount of time we want to wait
duration time_to_wait
---

# Part 2: the result, to be sent by the server upon completion

# How much time we waited
duration time_elapsed

# How many updates we provided along the way
uint32 updates_sent

---

# Part 3: the feedback, to be sent periodically by the server during
# execution.

# The amount of time that has elapsed from the start
duration time_elapsed

# The amount of time remaining until we're done
duration time_remaining

```
