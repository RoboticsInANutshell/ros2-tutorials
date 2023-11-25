

# Installation 

## Installing ROS2 in your Ubuntu System

Installing ROS2 can be a bit tricky because the ROS2 version evolves over time. To ensure that this guide remains relevant for all upcoming versions, it's best to refer to the official ROS2 documentation for your distribution.

Humble ==> https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html


## Setting up your .bashrc (JUST READ)

Think of your `.bashrc` file as a personal assistant for your Linux command-line adventure. It empowers you to tailor your shell environment by setting variables, creating shortcuts, and even tweaking the appearance of your terminal prompt. Let's explore how to access and configure it. 

NOTE: You need to reload the terminal when you make changes to the .bashrc file. 

To access your `.bashrc` file, open a terminal and run:

```
gedit ~/.bashrc
```
This will open a text document. Your `.bashrc` file runs every time you open a terminal, automatically applying any configurations you've made. Here are some enhancements you can make by adding to your `.bashrc`:

#### Adding ROS Library Source
To streamline your ROS2 experience, add the following line to your .bashrc file:

```
source /opt/ros/<ros-distro>/setup.bash
```
This ensures that ROS2 libraries are sourced every time you open a terminal, allowing you to use ROS2 functionality seamlessly. Obviously replace `<ros-distro>`with your distro.

#### Automatically Source Your Workspace

```
source ~/your/workspace/install/setup.bash
``` 
This ensures that your workspace is always sourced, eliminating the need for manual sourcing. Keep in mind that when you build your workspace, you might need to source it again or reload the terminal to apply changes.

#### Alias - Your command line shortcuts

Alias is like having short text for a repeatative big command. for e.g.

```
alias ws="cd ~/your/workspace"
```
So when you load the terminal, just by pressing `ws` in the terminal, it will take you to your workspace. Use this as much as your can where you will find some repeatative commands. 

Following are some of the examples I use

```
alias sc="source install/setup.bash"
```
Use this to source your workspace by just typing `sc`. But you need to be in the root workspace for this to work. otherwise you can use the following if you are working on only one workpace
```
alias sc="source ~/your/workspace/install/setup.bash"
```

# Creating Workspace and Package 


For a detailed understanding of the Workspace and Packages, please refer to the following video for descriptive explaination. 

https://youtu.be/b6Gb8eCAoQg?si=a3NJvPL_FeNfMDxj


## Creating a Workspace

Start by creating a workspace folder. A good Practice is to give it a suffix as `ws` to identify it as a workspace. Now lets create a worksapce 

```
mkdir -p ros_tutorials_ws/src
```

Thats it. Your new workspace is ready. Hahaha, you expected more?

## Creating a Custom Package

Now first go to your `src` folder 

```
cd ~/ros_tutorials_ws/src/
```

then run the following command to create a custom package with a package name `ros_basics`

```
ros2 pkg create --build-type ament_python ros_basics
```
This will create a package named `ros_basics`

What is `--build-type`?: It's a crucial option that determines how your package will be built. It allows you to choose between two build types:

- `ament_python`: This option is used when you want to create a package that primarily contains Python code. It configures your package to use Python files for its implementation.
- `ament_cmake`: On the other hand, if you choose ament_cmake, your package will be set up to use C++ files for its implementation. This option is ideal for developers who prefer to work with C++ for their ROS 2 packages.

When you create your custom package with ament_python, you'll notice some key differences compared to ament_cmake:

- Your package will have a setup.py file instead of a CMakeLists.txt file.
- Your launch files will use a .py extension, like launch.py, rather than the .xml extension, such as launch.xml.

Which one to use?
This depends on your preferance. `ament_cmake` is based on ROS1 and if your are familier with editing the `cmakelist.txt` then use that. Otherwise use the `ament_python` as we will use it for this tutorial. 


## Downloading an Existing Package

Again go to your `src` folder 

```
cd ~/ros_tutorials_ws/src/
```
Then run the following to clone an existing Git Repository for ros tutorials from the offical website. 

```
git clone https://github.com/ros/ros_tutorials.git -b humble
```

Now you should have 2 folders in `/src` folder `ros_basics` and `ros_tutorials`, and hence now you have 2 packages in your workspace

## Building Workspace

Now building these packages is an important step. Hence now go back to your workspace root folder

```
cd ~/ros_tutorials_ws/
```
and then run this

```
colcon build 
```

The above command should build both the packages for you and now inside your workspace your will notice more folders are added `logs` `install` `builds`. 

If you want to build just one package in the workspace then use package select


```
colcon build --pacakges-select ros_basics
```

This will just build the `ros_basics` package and not the `ros_tutorials`package

Also there is one more parameter, which will be useful in the future which is
```
colcon build --symlink-install
```
This command will make sure that you do not need to build the package everytime you make the changes to the python file. Although, when you make a new launch or executable files, then building the package irrespective manually is a good practise. 


# Creating your first Publisher and Subscriber

For visual representation please refer to this Youtube video --> 

## Creating a publisher

go to your package `ros_basics` by the following command

```
cd ~/ros_tutorials_ws/src/ros_basics/python_package
```

Now create a your python file with a name `test_publisher.py` using the following command

```
touch test_publisher.py
```
Now open the file in your favourite text editor (e.g. visual studio) and put the following content into it

```

#!/usr/bin/env python3


import rclpy
import random
from geometry_msgs.msg import Point
from rclpy.node import Node



class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("Pub_Node")
        self.pub = self.create_publisher(Point, 'communication',10)
        self.timer_period = 0.1  # 1 second
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        coordinate = Point()
        coordinate.x= random.uniform(1,100)
        coordinate.y= random.uniform(1,100)
        coordinate.z = random.uniform(1,100)  
        self.pub.publish(coordinate)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


```



This code is a Python script for a ROS 2 (Robot Operating System 2) publisher node that publishes random 3D points (x, y, and z coordinates) to a ROS 2 topic. 

For Detailed Explaination refer [Publisher Explained](extras/publisher_explained.md)

## Create Subscriber

go to your package `ros_basics` by the following command

```
cd ~/ros_tutorials_ws/src/ros_basics/python_package
```

Now create a your python file with a name `test_subscriber.py` using the following command

```
touch test_subscriber.py
```
Now open the file in your favourite text editor (e.g. visual studio) and put the following content into it
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('Sub_Node')
        self.sub = self.create_subscription(Point, 'communication',self.callback, 10)
        self.sub


    def callback(self,num):
        print(num)

def main (args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

    pass    
    


if __name__=='__main__':
    main()
```



This code is a Python script for a ROS 2 subscriber node that subscribes to the 'communication' topic and prints the received messages of type `geometry_msgs.msg.Point`. 


For Detailed explaination please refer to
[Subscriber Explained](extras/subscriber_explained.md)

