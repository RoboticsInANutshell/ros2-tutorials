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



This code is a Python script for a ROS 2 (Robot Operating System 2) publisher node that publishes random 3D points (x, y, and z coordinates) to a ROS 2 topic. Let's break down the code and explain each part in detail:

1. Shebang and Import Statements:
   ```python
   #!/usr/bin/env python3
   ```
   This line is a shebang, specifying the interpreter to be used for running the script. In this case, it's using Python 3. The `#!/usr/bin/env` part allows you to run the script without specifying the full path to the Python interpreter.

   ```python
   import rclpy
   import random
   from geometry_msgs.msg import Point
   from rclpy.node import Node
   ```

   - `rclpy` is a Python library for ROS 2 that provides the Python interface for working with ROS 2 nodes and communication.
   - `random` is a Python standard library module used to generate random numbers.
   - `geometry_msgs.msg` provides the `Point` message type, which is used to represent 3D coordinates.
   - `rclpy.node` provides the `Node` class, which is a fundamental building block for ROS 2 nodes.

2. Class Definition: `MinimalPublisher`
   ```python
   class MinimalPublisher(Node):
   ```
   This code defines a class called `MinimalPublisher`, which inherits from the `Node` class provided by `rclpy`. This class will represent the ROS 2 publisher node.

   - `def __init__(self):` defines the constructor for the class. This method is executed when an object of this class is created.

     - `super().__init__("Pub_Node")` initializes the node with the name "Pub_Node."
     - `self.pub = self.create_publisher(Point, 'communication', 10)` creates a publisher that publishes messages of type `Point` on the 'communication' topic with a queue size of 10.
     - `self.timer_period = 0.1` sets the timer period to 0.1 seconds (i.e., the `timer_callback` method will be called every 0.1 seconds).
     - `self.timer = self.create_timer(self.timer_period, self.timer_callback)` creates a timer that triggers the `timer_callback` method at the specified interval.

   - `def timer_callback(self):` defines the callback method for the timer. This method is executed when the timer triggers.

     - `coordinate = Point()` creates an instance of the `Point` message type to store the 3D coordinates.
     - `coordinate.x = random.uniform(1, 100)` generates a random floating-point number between 1 and 100 for the x-coordinate.
     - `coordinate.y = random.uniform(1, 100)` generates a random floating-point number between 1 and 100 for the y-coordinate.
     - `coordinate.z = random.uniform(1, 100)` generates a random floating-point number between 1 and 100 for the z-coordinate.
     - `self.pub.publish(coordinate)` publishes the generated `coordinate` on the 'communication' topic.

3. `main` Function:
   ```python
   def main(args=None):
   ```
   This function is the entry point for the script.

   - `rclpy.init(args=args)` initializes the ROS 2 client library. It takes command-line arguments as input (typically, `args=None`), and it's the first step in starting a ROS 2 node.

   - `minimal_publisher = MinimalPublisher()` creates an instance of the `MinimalPublisher` class, which represents the ROS 2 node.

   - `rclpy.spin(minimal_publisher)` starts the ROS 2 event loop and allows the node to process callbacks. This function doesn't return until the node is shut down.

   - `minimal_publisher.destroy_node()` explicitly destroys the node when it's no longer needed.

   - `rclpy.shutdown()` shuts down the ROS 2 client library.

4. Entry Point:
   ```python
   if __name__ == '__main__':
       main()
   ```
   This conditional block ensures that the `main` function is executed when the script is run directly, not when it's imported as a module.

In summary, this code defines a ROS 2 publisher node that periodically publishes random 3D points (x, y, and z coordinates) on the 'communication' topic. It uses the ROS 2 Python library, `rclpy`, and the `geometry_msgs.msg.Point` message type to achieve this. The node continues running until it is explicitly shut down.

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



This code is a Python script for a ROS 2 subscriber node that subscribes to the 'communication' topic and prints the received messages of type `geometry_msgs.msg.Point`. Let's break down the code and explain each part in detail:

1. Shebang and Import Statements:
   ```python
   #!/usr/bin/env python3
   ```
   This line is a shebang, specifying the interpreter to be used for running the script. In this case, it's using Python 3.

   ```python
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Point
   ```
   - `rclpy` is a Python library for ROS 2 that provides the Python interface for working with ROS 2 nodes and communication.
   - `rclpy.node` provides the `Node` class, which is a fundamental building block for ROS 2 nodes.
   - `geometry_msgs.msg` provides the `Point` message type, which is used to represent 3D coordinates.

2. Class Definition: `MinimalSubscriber`
   ```python
   class MinimalSubscriber(Node):
   ```
   This code defines a class called `MinimalSubscriber`, which inherits from the `Node` class provided by `rclpy`. This class will represent the ROS 2 subscriber node.

   - `def __init__(self):` defines the constructor for the class. This method is executed when an object of this class is created.

     - `super().__init__('Sub_Node')` initializes the node with the name "Sub_Node."

     - `self.sub = self.create_subscription(Point, 'communication', self.callback, 10)` creates a subscriber that listens to messages of type `Point` on the 'communication' topic with a queue size of 10. When a message is received on this topic, the provided `self.callback` method is called.

     - `self.sub` is assigned to the created subscription, but it doesn't serve any specific purpose in this context.

   - `def callback(self, num):` defines the callback method for the subscriber. This method is executed when a message is received on the 'communication' topic.

     - `print(num)` simply prints the received message, which is an instance of the `Point` message type. This will display the 3D coordinates of the received point.

3. `main` Function:
   ```python
   def main(args=None):
   ```
   This function is the entry point for the script.

   - `rclpy.init(args=args)` initializes the ROS 2 client library. It takes command-line arguments as input (typically, `args=None`), and it's the first step in starting a ROS 2 node.

   - `minimal_subscriber = MinimalSubscriber()` creates an instance of the `MinimalSubscriber` class, which represents the ROS 2 node.

   - `rclpy.spin(minimal_subscriber)` starts the ROS 2 event loop and allows the node to process incoming messages and callbacks. This function doesn't return until the node is shut down.

   - `minimal_subscriber.destroy_node()` explicitly destroys the node when it's no longer needed.

   - `rclpy.shutdown()` shuts down the ROS 2 client library.

4. Entry Point:
   ```python
   if __name__ == '__main__':
       main()
   ```
   This conditional block ensures that the `main` function is executed when the script is run directly, not when it's imported as a module.

In summary, this code defines a ROS 2 subscriber node that listens to the 'communication' topic, and when it receives messages of type `geometry_msgs.msg.Point`, it prints the received coordinates. The node continues running until it is explicitly shut down.






