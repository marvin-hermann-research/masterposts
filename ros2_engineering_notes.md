# ROS2 Engineering Notes - My Knowledge Stack

---

ROS is a system that enables the control, maintenance, and design of individual components of one or multiple robotic systems via so-called **nodes**, which can be distributed over multiple computers

## Setting up ROS 2

Install system-wide: `ros-humble-desktop`  
ROS 2 **must always** be sourced in the terminal before use:

```bash
source /opt/ros/humble/setup.bash
```

=> Add this command to your shell startup file to source it automatically:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## ROS 2 Structure

ROS uses a **workspace**, which organizes all ROS projects into **packages** under the `src/` directory. Each package contains **nodes**, which can be written in **C++** or **Python**, as well as other executable program code

=> Within the ROS project folder, also create the **Python project** folder

Example folder structure:

```lua
ros_workspace/
└─ src/
   └─ bipedal_robot_pkg/
      ├─ package.xml
      ├─ setup.py
      ├─ setup.cfg
      ├─ resource/
      └─ bipedal_robot_pkg/         <--- IMPORTANT: subfolder with package name
          ├─ __init__.py
          ├─ battery_monitor_node.py
          ├─ imu_sensor_node.py
          ├─ laser_sensor_node.py
          └─ ... other node scripts
```

## Building the Workspace

You must build the workspace whenever:

- New `.py` scripts are created
- Changes are made to `package.xml` or `setup.py`
- A new ROS package is created

This is similar to compiling, and also necessary when creating or installing scripts for the first time

=> To build run `colcon build` in the workspace directory

After each build, environment variables must be reloaded (otherwise nodes will not be visible):

```bash
source ~/WORKSPACENAME/install/setup.bash
```

This reloads the specific workspace

## Creating a New Package

Inside the workspace run:

```bash
ros2 pkg create --build-type ament_python PACKAGENAME
```

You can create either Python or C++ packages using:

- `ament_python`
- `ament_cmake`

Each package contains **setup.py** and **package.xml** files

### setup.py

Acts as the executable for the `colcon` build command

```python
from setuptools import setup

package_name = 'temperature_sensor_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marvin',
    maintainer_email='marvin@example.com',
    description='A simple temperature sensor node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = temperature_sensor_pkg.temperature_publisher:main'
        ],
    },
)
```

### package.xml

Contains all dependencies related to packages, versions, imports, and author info

```xml
<package format="3">
  <name>temperature_sensor_pkg</name>
  <version>0.1.0</version>
  <description>A ROS 2 package for a temperature sensor</description>

  <maintainer email="marvin@example.com">Marvin</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```

## Creating a Node

Nodes are simple Python or C++ files placed inside packages

**Node classes must import ROS and inherit from Node:**

```python
import rclpy  # ROS Client Library for Python
from rclpy.node import Node

class NodeName(Node):
    def __init__(self):
        super().__init__('YOUR_NODE_NAME')
```

**Nodes must be registered in `setup.py`:**  

This is required for the node to be accessible via `ros2 run` and discoverable by other ROS tools

**=> All nodes whose data you want to test, observe, or start at any time should be callable via a separate `main()` function**

Manual starting or importing does not require registration

**ROS nodes are launched using the following syntax**

```bash
ros2 run <package_name> <entry_point_name>
```

Therefore, **entry_points** must be added in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'RUN_COMMAND_NAME = ROS_PACKAGE_NAME.NODE_FILE_NAME:ENTRY_METHOD_NAME'
    ],
},
```

**Example**

In `setup.py`:

```python
entry_points={
    'console_scripts': [
        'imu_sensor = bipedal_robot_pkg.imu_sensor_node:main',
    ],
},
```

In terminal:

```bash
ros2 run bipedal_robot_pkg imu_sensor
```

The Node also requires an entry point method named conventionally `main` outside the class:

```python
def main(args=None):
    rclpy.init(args=args)           # 1) Initialize the ROS 2 system
    node = MyNodeClass()            # 2) Create your node instance
    rclpy.spin(node)                # 3) Keep node alive and process messages (subscribers, timers, services...)
    node.destroy_node()             # 4) Clean up the node (e.g. close threads, free memory)
    rclpy.shutdown()                # 5) Shutdown the ROS 2 system cleanly
```

## Naming Conventions

- Packages: `snake_case_pkg`
- Nodes: `snake_case.py` (standard Python style)

## Publisher & Subscriber Nodes

In ROS, nodes can have one or both roles depending on the use case

### Publisher Node

- The node sends data at regular intervals on a **topic**

Create a publisher inside the node's `__init__` method and import the topic message type:

```python
self._publisher = self.create_publisher(TOPIC_MSG_TYPE, 'TOPIC_NAME', QUEUE_SIZE)
```

### Subscriber Node

- The node listens continuously to one or more **topics** and reacts to events

Create a subscriber inside the node's `__init__` method and import the topic message type:

```python
self._subscription = self.create_subscription(TOPIC_MSG_TYPE, 'TOPIC_NAME', self.callback_method, QUEUE_SIZE)
```

#### Subscriber callback method

The received message is passed as the second parameter

```python
def listener_callback(self, msg):
    # process msg.data
```

**Whenever possible, nodes should adhere to the single-responsibility principle.**

### QUEUE_SIZE

Relates to ROS's QoS (Quality of Service). It defines how many messages are buffered on the topic to handle possible packet loss

|Queue Size|Meaning|
|---|---|
|1|Only the newest message is buffered – very reactive|
|10|Moderate buffer size – default|
|1000|For high-frequency sensor streams (e.g. camera)|

## Topics

Topics are persistent open **channels** each corresponding to a single relevant recurring event

Each node can subscribe to any topic stream published within the ROS network.

### Topic Naming Convention: component/dataName

**IMPORTANT:** Normally, there should only be **ONE** publisher per topic, unless explicit redundancy is required
If multiple publishers share a topic, signals **MUST** be identical

### Listing Topics

- List active topics

```bash
ros2 topic list
```

- Print all messages on a topic:

```bash
ros2 topic echo /topic_name
```

## ROS 2 Topic Message Types (Best-of List)

|Type|Import|Description|
|---|---|---|
|`String`|`from std_msgs.msg import String`|Simple text|
|`Bool`|`from std_msgs.msg import Bool`|True/False (e.g., "Tipped?")|
|`Int32`, `Int64`, `Float32`, `Float64`|`from std_msgs.msg import Float32` etc.|Simple numeric values|
|`Float32MultiArray`|`from std_msgs.msg import Float32MultiArray`|Multiple floats as list (e.g., 3 laser sensors, IMU)|
|`LaserScan`|`from sensor_msgs.msg import LaserScan`|Comprehensive laser scan data including angles, ranges, intensities, range limits, timing info|
|`Imu`|`from sensor_msgs.msg import Imu`|Inertial measurement data: orientation (quaternion), angular velocity, linear acceleration, covariance matrices|
|`BatteryState`|`from sensor_msgs.msg import BatteryState`|Battery status: voltage, current, charge, capacity, percentage, power supply status|
|`Twist`|`from geometry_msgs.msg import Twist`|Motion info: linear (x,y,z) and angular (roll, pitch, yaw) velocity|
|`Pose`|`from geometry_msgs.msg import Pose`|Position and orientation of an object in space|
|`PoseStamped`|`from geometry_msgs.msg import PoseStamped`|Pose with timestamp and coordinate frame metadata|
|`JointState`|`from sensor_msgs.msg import JointState`|Robot joint states: names, positions, velocities, effort|

## Action Nodes

When publisher/subscriber nodes need to perform a specific **task**, they do so via _action nodes_, e.g., **navigate_to_target_node**.

**Action nodes:**

- Are single purpose
- Are asynchronous
- Provide feedback
- Can optionally be canceled

### Using Action Nodes

Action nodes consist of three components:

1. **Action Type**
2. **The action node itself**
3. **The action topic**

#### Action Types

Python classes generated from `.action` files. Predefined ones include:

- `Fibonacci`
- `FollowJointTrajectory` (MoveIt)
- `NavigateToPose` (Nav2)
- `Dock`, `Undock` etc.

##### Structure of a `.action` file

`WalkPattern.action`
```plain
# === GOAL === Task assigned to the server
string pattern_name
int32 step_count

---
# === RESULT === Result returned by the server
bool success
string message

---
# === FEEDBACK === Intermediate feedback
int32 percent_complete
string current_step_description
```

##### Using the action in code

```python
from custom_interfaces.action import WalkPattern

goal = WalkPattern.Goal()
goal.pattern_name = "ZigZag"
goal.step_count = 10

feedback = WalkPattern.Feedback()
feedback.percent_complete = 40
feedback.current_step_description = "Left leg lifted"

result = WalkPattern.Result()
result.success = True
result.message = "Pattern completed"
```

#### Action Node structure

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from PACKAGE.action import WalkPattern  # ActionType

class PatternExecutor(Node):
    def __init__(self):
        super().__init__('pattern_executor')
        self._action_server = ActionServer(
            self,                    # Node instance
            WalkPattern,             # ActionType
            'walk_pattern_executor', # Action topic
            self.execute_callback    # Execution callback
        )

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request          # Get assigned task
        feedback = WalkPattern.Feedback()   # Prepare feedback context
        # ... during the task:
        goal_handle.publish_feedback(feedback)
        # ... on completion:
        goal_handle.succeed()
        result = WalkPattern.Result()
        return result                      # Return result object

        # Optional callbacks:
        # goal_callback for accepting/rejecting goals
        # cancel_callback to handle cancel requests
```
#### Action Topics

Created when the action server is instantiated. Includes 4 subtopics:

```plain
/walk_pattern_executor/_goal
/walk_pattern_executor/_feedback
/walk_pattern_executor/_result
/walk_pattern_executor/_cancel
```


=> There is no need for manual topic subscription; communication is handled through `ActionClient` and `ActionServer` interfaces

#### Using Action Nodes

Conventionally, a regular node creates a **Goal** and sends it to the action node via the **ActionClient** using `send_goal_async()`, waiting for the response.

```python
from custom_interfaces.action import WalkPattern
from rclpy.action import ActionClient

class SensorSubscriberNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.sub = self.create_subscription(String, 'start_pattern', self._callback, 10)
        self.client = ActionClient(self, WalkPattern, 'walk_pattern_executor')

    def _callback(self, msg):
        goal_msg = WalkPattern.Goal()
        goal_msg.pattern_name = msg.data
        self.client.wait_for_server()
        self.future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self.future.add_done_callback(self._response_callback)
```

## Useful ROS 2 Run Commands

- `ros2 topic list` — lists all active topics
- `ros2 topic echo /TOPIC_NAME` — continuously outputs the topic’s messages
- `rqt graph` — opens GUI to visualize all active nodes and topics

## Logging in ROS

It is standard to use the ROS Python API’s built-in **get_logger()** for status messages instead of print statements.  
Logging should be sparing and used in typical situations as follows:

|Phase|Log Level|Example|
|---|---|---|
|Initialization|`info`|`"Left leg subscriber initialized"`|
|Incoming Data|`debug` or `info`|`"Received instruction: [...]"`|
|Implausible Sensor Data|`warn`|`"IMU angle > 90°, robot might be tipping"`|
|Battery Low|`error`|`"Battery voltage critically low"`|
|Hardware Connection Fail|`fatal`|`"No response from motor driver – aborting"`|

Example usage:

```python
self.get_logger().info("Left leg active")
# Output:
# [INFO] [1687318301.492038902] [left_leg]: Left leg active
```

## Integration of Sensor Data

### Header

```python
from std_msgs.msg import Header  # import
msg.header = Header()            # create header instance
```

The ROS2 header contains metadata used for synchronization and coordination of sensor data, including:

- **stamp:** Timestamp obtained from the clock:

```python
msg.header.stamp = self.get_clock().now().to_msg()  # current ROS timestamp
```

- **frame_id:** Coordinate frame where the data was generated (e.g., `"laser_frame"`, `"base_link"`, `"map"`):

```python
msg.header.frame_id = "laser_frame"
```

### Geometry Data

```python
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, Twist
```

ROS2 geometry messages provide:

- **Point** (x, y, z)
- **Vector3** (x, y, z)
- **Quaternion** (w, x, y, z)
- **Pose** (combination of Point and Quaternion)
- **Twist** (combination of linear and angular velocities)

Example usage (similar to Unity style vector creation):

```python
# Quaternion: Orientation (rotation) in space
q = Quaternion()
q.w = 1.0
q.x = 0.0
q.y = 0.0
q.z = 0.0

# Vector3: Vector with 3 components, e.g. velocity or acceleration
v = Vector3()
v.x = 0.5
v.y = -0.2
v.z = 0.1

# Point: Position in space
p = Point()
p.x = 1.0
p.y = 2.0
p.z = 3.0

# Pose: Position + orientation (combination of Point and Quaternion)
pose = Pose()
pose.position = p
pose.orientation = q

# Twist: Motion (translation + rotation)
twist = Twist()
twist.linear = v          # linear velocity (Vector3)
twist.angular = Vector3() # angular velocity (zero vector here)
twist.angular.x = 0.0
twist.angular.y = 0.0
twist.angular.z = 0.1
```

## Motion Sequence YAMLs

Motion sequences are stored in YAML files for example describing servo motor positions per frame.

These YAMLs can be generated efficiently via machine learning. This can be considered a starting point for venturing into "smart robotics"

**Convention:** Large sequences are placed in individual YAMLs, and a main YAML loads all others:

`all_patterns.yaml`:

```yaml
patterns:
  walk_forward: "patterns/walk_forward.yaml"
  arm_wave: "patterns/arm_wave.yaml"
```

The main YAML is loaded by a node, and specific pattern YAMLs are loaded on demand.

### YAML structure example

`walk_forward.yaml`:

```yaml
# Motion pattern: "walk_forward"
walk_forward:
  - time: 0.0       # timestamp in seconds
    joints:
      hip: 30 
      knee: 60
  - time: 0.5
    joints:
      hip: 45
      knee: 80
  - time: 1.0
    joints:
      hip: 60
      knee: 60
```

### Using YAML

Import `yaml`, load in the action node’s `__init__` and convert to dictionary. In the callback, retrieve pattern name from goal, check existence, convert pattern with timestamps to a list, and execute sequentially or in parallel.

```python
import yaml

# in __init__:
with open('patterns.yaml', 'r') as file:
    self.patterns = yaml.safe_load(file)

# example: self.patterns is now a dict with keys "walk_forward" and others

async def execute_callback(self, goal_handle):
    self.start_time = time.time()
    self.goal_handle = goal_handle
    self.steps = self.patterns[goal_handle.request.pattern_name]
    self.current_step_idx = 0

    # check every 0.05s if next instruction should be executed
    self.timer = self.create_timer(0.05, self.timer_callback)

    # wait until all steps are executed
    while self.current_step_idx < len(self.steps):
        await asyncio.sleep(0.01)  # small pause for loop

    self.timer.cancel()

    self.goal_handle.succeed()
    result = WalkPattern.Result()
    result.success = True
    result.message = "Pattern finished"
    return result

def timer_callback(self):
    elapsed = time.time() - self.start_time

    while self.current_step_idx < len(self.steps) and self.steps[self.current_step_idx]['time'] <= elapsed:
        step = self.steps[self.current_step_idx]
        # set joint positions here
        self.execute_joints(step['joints'])

        self.current_step_idx += 1

def execute_joints(self, joints_dict):
    # Code to send joint angles to hardware
    pass
```

## Implementing Py_trees Nodes in Ros

In robotic systems that rely on both Behavior Trees and ROS 2, it is essential to utilize the **py_trees_ros2** extension in addition to the base **py_trees library**
This extension ensures proper synchronization between the behavior tree's ticking mechanism and the ROS 2 communication layer, such as topic updates and callbacks
## Core Integration Principles

While **Condition Nodes** and **Action Nodes** continue to follow the conventional `py_trees.Behaviour` interface, the *architecture-level components*, such as the behavior tree factory and the ROS 2 execution entry point, are adapted to integrate with `py_trees_ros2`

## Py_trees and ROS2 Workflow

```bash
├── [1] py_trees_condition_node           # py_trees.Behaviour
│     ├── Evaluates internal state via writer interface
│     └── Returns status (SUCCESS, FAILURE, RUNNING)
│
├── [2] py_trees_action_node              # py_trees.Behaviour
│     ├── Receives instantiated ROS 2 publisher node
│     ├── Publishes messages the recieved node
│     └── Returns status (RUNNING → SUCCESS)
│
├── [3] ros2_publisher_node         # rclpy.Node
│     ├── Instantiated during ROS 2 system initialization
│     └── Passed into py_trees nodes to decouple logic
│
├── [4] ros2_listener_node          # rclpy.Node
│     ├── Subscribes to sensor topics
│     └── Evaluates data and updates shared state
│
└── [5] ros2_action_client          # rclpy.Node
      ├── Sends e.g. MoveBaseGoal to action servers
      └── Processes asynchronous results from server

```

## Condition Node Setup

>Condition nodes are implemented using standard `py_trees` behavior classes and interact with a blackboard write interface

A ROS-based sensor publisher node may directly implement a blackboard write interface **only if** its local sensor data is sufficient to determine the corresponding condition in the behavior tree.

In contrast, if the condition logic requires **data from multiple sensor sources**, a dedicated **listener-writer abstraction** is introduced.
This component subscribes to the relevant topics, performs optional preprocessing, and updates the blackboard with a logically consistent state variable.

```python
import py_trees
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, BatteryState, Imu
from std_msgs.msg import Header

# Creating the publisher Node as an evaluator
class CanWalkEvaluator(Node):
    def __init__(self):
        super().__init__("can_walk_evaluator")
        self._init_subscribers()
        
		# registering the publisher node to the same key 
		self._init_blackboard()

    def _init_blackboard(self):
        self._bb = py_trees.blackboard.Client(name="CanWalkWriterClient")
        self._bb.register_key(
	        key="can_walk",
	        access=py_trees.common.Access.WRITE
	    )

    def _init_subscribers(self):
        self._imu_sensor_subscriber = self.create_subscription(
            Imu,
            "imu_sensor_data",
            self._imu_listener_callback,
            10
        )
        self.get_logger().info("IMU sensor subscriber initialized.")

        self._laser_sensor_subscriber = self.create_subscription(
            LaserScan,
            "laser_sensor_data",
            self._laser_listener_callback,
            10
        )
        self.get_logger().info("Laser sensor subscriber initialized.")

        self._battery_monitor_subscriber = self.create_subscription(
            BatteryState,
            "battery_monitor_data",
            self._battery_listener_callback,
            10
        )
        self.get_logger().info("Battery monitor subscriber initialized.")

    def _imu_listener_callback(self, data: Imu):
        return

    def _laser_listener_callback(self, data: LaserScan):
        return

    def _battery_listener_callback(self, data: BatteryState):
        return
```

## Action Node Setup 

> Action nodes remain subclasses of `py_trees.Behaviour`, but rely on external ROS 2 publisher nodes. These publishers are passed as parameters and invoked during execution.

This design **avoids redundant instantiations** of publisher interfaces and ensures that publishing responsibilities are testable, modular, and reusable outside the behavior tree context.

```python
import py_trees

class MoveForwardAction(py_trees.behaviour.Behaviour):
    def __init__(self, publisher_node):
        super().__init__(name="MoveForwardAction")
        self.publisher_node = publisher_node
        self.sent = False

    def update(self):
        if not self.sent:
            self.publisher_node.send_forward_command()
            self.sent = True
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS
```

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MoveForwardPublisher(Node):
    def __init__(self):
        super().__init__('move_forward_publisher')
        self._publisher = self.create_publisher(String, '/movement_command', 10)

    def send_forward_command(self):
        msg = String()
        msg.data = "forward"
        self._publisher.publish(msg)
```

# Final Words

This post evolves as I evolve. I will continuously refine and expand it as I deepen my understanding. Feedback and suggestions are always welcome!

---

#  Change Log

| Version | Date       | Changes                                                                                                                                                              |
| ------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1.0.2   | 2025-07-19 | - Added py_trees Action Node Setup section<br>- Added Core Integration Principles section<br>- Added Py_trees and ROS2 Workflow section<br> |
| 1.0.1   | 2025-07-17 | Added py_trees Condition Node setup                                                                                                                                  |
| 1.0.0   | 2025-07-15 | Added py_trees integration chapter                                                                                                                                   |
