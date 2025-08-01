# ROS2 Engineering Notes - My Knowledge Stack

## Table of Contents

- [Setting up ROS 2](#setting-up-ros-2)
- [ROS 2 Structure](#ros-2-structure)
- [Building the Workspace](#building-the-workspace)
- [Creating a New Package](#creating-a-new-package)
  - [setup.py](#setuppy)
  - [package.xml](#packagexml)
- [Creating a Node](#creating-a-node)
- [Naming Conventions](#naming-conventions)
- [Publisher & Subscriber Nodes](#publisher--subscriber-nodes)
  - [Publisher Node](#publisher-node)
  - [Subscriber Node](#subscriber-node)
    - [Subscriber callback method](#subscriber-callback-method)
  - [QUEUE_SIZE](#queue_size)
- [Topics](#topics)
  - [Topic Naming Convention: component/dataName](#topic-naming-convention-componentdataname)
  - [Listing Topics](#listing-topics)
- [ROS 2 Topic Message Types (Best-of List)](#ros-2-topic-message-types-best-of-list)
- [Action Nodes](#action-nodes)
  - [Using Action Nodes](#using-action-nodes)
    - [Action Types](#action-types)
      - [Structure of a .action file](#structure-of-a-action-file)
      - [Using the action in code](#using-the-action-in-code)
    - [Action Node structure](#action-node-structure)
    - [Action Topics](#action-topics)
    - [Using Action Nodes (Client Side)](#using-action-nodes-client-side)
- [Useful ROS 2 Run Commands](#useful-ros-2-run-commands)
- [Logging in ROS](#logging-in-ros)
- [Integration of Sensor Data](#integration-of-sensor-data)
  - [Header](#header)
  - [Geometry Data](#geometry-data)
- [Motion Sequence YAMLs](#motion-sequence-yamls)
  - [YAML structure example](#yaml-structure-example)
  - [Using YAML](#using-yaml)
- [Implementing Py_trees Nodes in Ros](#implementing-py_trees-nodes-in-ros)
- [Core Integration Principles](#core-integration-principles)
- [Py_trees and ROS2 Workflow](#py_trees-and-ros2-workflow)
- [Condition Node Setup](#condition-node-setup)
	- [Setting Conditions via Console](#setting-conditions-via-console)
- [Action Node Setup](#action-node-setup)
- [Tree Factory Setup in ROS2](#tree-factory-setup-in-ros2)
  - [Example: Locomotion Subtree](#example-locomotion-subtree)
- [Application Class-Based Initialization of ROS 2 Systems](#application-class-based-initialization-of-ros-2-systems)
	- [Installing py_trees_ros for ROS 2 ](#installing-py-trees-ros-for-ros-2)
	- [Application Class Structure and Responsibilities](#application-class-structure-and-responsibilities)
	- [Detailed Explanation](#detailed-explanation)
    - [Wrapping the Behavior Tree in a ROS 2-Compatible Structure](#wrapping-the-behavior-tree-in-a-ros-2-compatible-structure)
    - [Registering Nodes with the ROS 2 Executor](#registering-nodes-with-the-ros-2-executor)
- [Entry Point](#entry-point)
- [Usage of `ament_cmake`](#usage-of-ament_cmake)
  - [Migrating from `ament_python` to `ament_cmake`](#migrating-from-ament_python-to-ament_cmake)
  - [Example Conversion](#example-conversion)
    - [`setup.py` (Before Migration)](#setuppy-before-migration)
    - [Executable Rights for Python Scripts](#executable-rights-for-python-scripts)
      - [Option 1 – With Git](#option-1--with-git)
      - [Option 2 – Without Git](#option-2--without-git)
    - [Final `CMakeLists.txt` (After Migration)](#final-cmakeliststxt-after-migration)
- [Setting Conditions via Console Using ROS 2 Services](#setting-conditions-via-console-using-ros-2-services)
  - [ROS 2 Service Setup](#ros-2-service-setup)
  - [Runtime Usage from Console](#runtime-usage-from-console)
  - [Required Package Configuration](#required-package-configuration)
- [Final Words](#final-words)
- [Change Log](#change-log)

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

You can create either Python packages or C++ *and* Python packages using:

- `ament_python`
- `ament_cmake`

*for now I will only focus on ament_python. A cmake section can be found later on*

An ament_python package contains **setup.py** and **package.xml** files

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

| Type                                   | Import                                       | Description                                                                                                     |
| -------------------------------------- | -------------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| `String`                               | `from std_msgs.msg import String`            | Simple text                                                                                                     |
| `Bool`                                 | `from std_msgs.msg import Bool`              | True/False (e.g., "Tipped?")                                                                                    |
| `Int32`, `Int64`, `Float32`, `Float64` | `from std_msgs.msg import Float32` etc.      | Simple numeric values                                                                                           |
| `Float32MultiArray`                    | `from std_msgs.msg import Float32MultiArray` | Multiple floats as list (e.g., 3 laser sensors, IMU)                                                            |
| `LaserScan`                            | `from sensor_msgs.msg import LaserScan`      | Comprehensive laser scan data including angles, ranges, intensities, range limits, timing info                  |
| `Imu`                                  | `from sensor_msgs.msg import Imu`            | Inertial measurement data: orientation (quaternion), angular velocity, linear acceleration, covariance matrices |
| `BatteryState`                         | `from sensor_msgs.msg import BatteryState`   | Battery status: voltage, current, charge, capacity, percentage, power supply status                             |
| `Twist`                                | `from geometry_msgs.msg import Twist`        | Motion info: linear (x,y,z) and angular (roll, pitch, yaw) velocity                                             |
| `Pose`                                 | `from geometry_msgs.msg import Pose`         | Position and orientation of an object in space                                                                  |
| `PoseStamped`                          | `from geometry_msgs.msg import PoseStamped`  | Pose with timestamp and coordinate frame metadata                                                               |
| `JointState`                           | `from sensor_msgs.msg import JointState`     | Robot joint states: names, positions, velocities, effort                                                        |

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

This implementation introduces a modular and scalable YAML-based motion execution system that decouples behavior triggering from trajectory execution. Motion patterns are externalized into YAML files and dynamically loaded and executed in response to behavior tree signals

The functionality is split across four methods:

1. **`__init__()`**  
    At initialization, a pattern index file (`patterns.yaml`) is parsed into a dictionary `self._patterns`, mapping behavior names (e.g., `"walk_forward"`, `"idle"`) to the corresponding YAML file paths. A periodic timer (`~20 Hz`) is also created to enable real-time motion sequencing
    
2. **`_walk_forward_callback()`**  
    This method is triggered when the `WalkForwardBehaviour` BT node becomes active. It delegates execution to `_load_and_start_pattern()` by passing the name `"walk_forward"`
    
3. **`_load_and_start_pattern()`**  
    Loads the actual motion sequence for a given behavior from disk. The selected YAML file is parsed into a time-ordered list of steps, each specifying joint targets and timestamps. The sequence is stored in `self._current_pattern`, and the playback state (start time, step index) is initialized
    
4. **`_pattern_timer_callback()`**  
    Runs every 50 ms and checks whether the next scheduled step in the motion pattern should be executed (based on its relative timestamp). If the current time has passed the step’s scheduled time, joint targets are sent to the actuators via `_execute_joints()`. This continues until the end of the pattern is reached
    
5. **`_execute_joints()`**  
    Converts joint dictionaries from the YAML step into `Float32MultiArray` ROS messages, and publishes them to the left and right leg controller topics. Each joint command is mapped explicitly to `hip_left`, `knee_left`, `hip_right`, and `knee_right`

```python
import yaml

def __init__(self):
    super().__init__("movement_controller")
        
    # Load the pattern index which maps behavior names to motion YAML files
    with open("patterns.yaml", "r") as file:
        self._patterns = yaml.safe_load(file)

    # Motion state
    self._current_pattern = None                # Currently loaded motion sequence
    self._pattern_start_time = None             # Timestamp when execution started
    self._pattern_step_index = 0                # Index of current step being executed

    # Periodic execution of motion steps (~20 Hz)
    self._timer = self.create_timer(0.05, self._pattern_timer_callback)


def _walk_forward_callback(self, instruction):
    """
    Triggered by WalkForwardBehaviour BT node.
    Starts execution of the 'walk_forward' pattern.
    """
    self._load_and_start_pattern("walk_forward")


def _load_and_start_pattern(self, pattern_name):
	#Load and prepare execution of a time-stamped motion pattern.
    path = self._patterns.get(pattern_name)
    if path is None:
        self.get_logger().error(f"Pattern '{pattern_name}' not found in index.")
        return

    with open(path, "r") as f:
        self._current_pattern = yaml.safe_load(f)[pattern_name]

    self._pattern_start_time = time.time()
    self._pattern_step_index = 0


def _pattern_timer_callback(self):
    """
    Periodic callback that checks and executes due pattern steps.

    Each step is executed when its timestamp has elapsed.
    When all steps are completed, the pattern is cleared.
    """
    if self._current_pattern is None or self._pattern_start_time is None:
        return

    elapsed = time.time() - self._pattern_start_time

    while (self._pattern_step_index < len(self._current_pattern) and
           self._current_pattern[self._pattern_step_index]["time"] <= elapsed):
        
        step = self._current_pattern[self._pattern_step_index]
        self._execute_joints(step["joints"])
        self._pattern_step_index += 1

    if self._pattern_step_index >= len(self._current_pattern):
        self._current_pattern = None
        self.get_logger().info("Pattern execution completed.")


def _execute_joints(self, joints):
	#Publish joint angle commands to both legs usinig a publisher.
    left_msg = Float32MultiArray()
    right_msg = Float32MultiArray()

    left_msg.data = [
        float(joints.get("hip_left", 0.0)),
        float(joints.get("knee_left", 0.0)),
    ]
    right_msg.data = [
        float(joints.get("hip_right", 0.0)),
        float(joints.get("knee_right", 0.0)),
    ]

    self._left_leg_publisher.publish(left_msg)
    self._right_leg_publisher.publish(right_msg)
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

class CanWalk(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="Can Walk?")
        self._blackboard = py_trees.blackboard.Client(name="CanWalkClient")
        self._blackboard.register_key(
            key="can_walk",
            access=py_trees.common.Access.READ # Read acces
        )

    def update(self):
        if self._blackboard.can_walk:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

```

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
        
		# registering the publisher node to the same key with write access
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

### Setting Conditions via Console

Publishing messages to a ROS2 topic via the console enables dynamic control of behavior tree conditions at runtime.  For this usecase an **Evaluator Class** can be used

The **Evaluator Class** subscribes to *one* topic to update a single condition flag.

For this use case, it is recommended to use the **`std_msgs/msg/Bool`** message type to represent binary conditions.

**Example: Publishing a boolean condition via console:**

```bash
ros2 topic pub /must_walk_command std_msgs/msg/Bool "{data: true}"
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

## Tree Factory Setup in ROS2

The general structure follows the conventions of vanilla `py_trees`, with additional ROS2 integration for communication nodes.

### Key Concepts

- All `py_trees` nodes (e.g., conditions, actions, composites like `Selector`, `Sequence`, `Parallel`) must be explicitly imported
- The factory is initialized with a dictionary of ROS2 communication nodes (e.g., publishers, service clients)
- Each subtree is built through a dedicated method:
  - Nodes are instantiated (receiving required ROS2 dependencies)
  - Then the behavior tree structure is assembled via standard `py_trees` composition

### Example: Locomotion Subtree

```python
import py_trees
from py_trees.composites import Selector
from your_condition_nodes import IsGroundedCondition
from your_action_nodes import MoveForwardAction

class TreeFactory:
    def __init__(self, ros_nodes: dict):
        """
        Dictionary containing all required ROS2 interfaces, e.g.:
        {
            "move_forward_publisher": MoveForwardPublisher(),
            "scan_target_client": ScanServiceClient(),
            ...
        }
        """
        self._ros_nodes = ros_nodes

    def create_locomotion_tree(self):
        # Create root composite
        root = Selector(
	        name="Locomotion Root",
			memory=True)

        # Instantiate leaf nodes
        is_grounded = IsGroundedCondition()  # Static condition, no ROS2 needed
        move_forward = MoveForwardAction(
        self._ros_nodes["move_forward_publisher"])

        # Compose the subtree
        root.add_children([is_grounded, move_forward])
        return root
```

## Application Class Based Initialization of ROS2 Systems

It is a common convention in ROS2-based development to encapsulate the initialization of the entire system within a single dedicated **application class**

While this architectural pattern is not inherently tied to the use of behavior trees or the `py_trees` framework, it provides a structured foundation that readily accommodates them

I introduce this concept at this stage because my primary use case focuses on the integration of `py_trees` within ROS2 systems, where the application class serves as a cohesive entry point for both robotic control logic and behavior orchestration

### Installing py_trees_ros for ROS 2

The `py_trees_ros` package provides ROS 2–specific extensions and behavior tree utilities for the `py_trees` framework. Despite some tutorials referring to a module named `py_trees_ros2`, **no such package exists**, all ROS 2 functionality is included within `py_trees_ros` version 2.x and above

To install it and all necessary components, run:

```bash
sudo apt update
sudo apt install \
  ros-humble-py-trees \
  ros-humble-py-trees-ros \
  ros-humble-py-trees-ros-interfaces \
  ros-humble-py-trees-ros-tutorials \
  ros-humble-py-trees-ros-viewer
```

Then build the workspace and source it. Now py_trees_ros ist importable

### Application Class Structure and Responsabilities

The Application class provides seven different services

1) **Initialize the ROS 2 runtime**
2) **Instantiate all required ROS 2 nodes**
3) **Construct the behavior tree via a factory class**
4) **Initialize the behavior tree using `py_trees_ros2`**
5) **Register all nodes with a multi-threaded ROS 2 executor**
6) **Define the main system loop (including behavior tree ticking)**
7) **Ensure graceful system shutdown**

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
import py_trees_ros2

from my_ros_nodes import MovePublisherNode
from tree_factory import TreeFactory

class BehaviorTreeApp:
    def __init__(self):
        # === 1. Initialize the ROS 2 runtime ===
        rclpy.init()

        # === 2. Instantiate ROS 2 nodes ===
        move_publisher_node = MovePublisherNode()
        self._nodes = {
            "move_publisher": move_publisher_node
        }

        # === 3. Construct the behavior tree via factory ===
        self._factory = TreeFactory(self.nodes)
        root = self._factory.create_locomotion_tree()

        # === 4. Initialize py_trees_ros2 wrapper ===
        self._tree = py_trees_ros2.BehaviourTree(
            root=root,
            node=self.nodes["move_publisher"],  # central or coordinating node
            name="LocomotionBT"
        )
        self._tree.setup(timeout=15)

        # === 5. Setup multi-threaded ROS 2 executor ===
        self._executor = MultiThreadedExecutor()
        for node in self.nodes.values():
            self.executor.add_node(node)

    # === 6. Launch the system loop ===
    def run(self):
        try:
            # Tick the behavior tree periodically
            self._tree.tick_tock(period_ms=100)

            # Spin the ROS 2 executor
            self._executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    # === 7. Clean shutdown procedure ===
    def shutdown(self):
        for node in self.nodes.values():
            node.destroy_node()
        rclpy.shutdown()
```

### Further Explaination

4) Wrapping the Behavior Tree in a ROS 2-Compatible Structure

```python
self.tree = py_trees_ros2.BehaviourTree(...)
```

This wraps a given behavior tree (specified via `root`) into a structure compatible with ROS 2

To make the tree operational within the ROS 2 ecosystem, one of your ROS nodes must act as the _host_ node. This host node provides the ROS execution context (e.g., clock, timers, logging) and **must** be a ROS node

Internally, `py_trees_ros2.BehaviourTree` sets up an `rclpy.Timer` and handles periodic ticking of the tree using `tick_tock(...)`
This ensures the tree is updated (ticked) in sync with other ROS components

The tree now behaves like any ROS node, integrated into the ROS event loop

5) Registering Nodes with the ROS 2 Executor

```python
`self.executor = MultiThreadedExecutor() for node in self.nodes.values():     self.executor.add_node(node)`
```

The executor is responsible for managing the callback queue, which includes all timers, subscriptions, and service callbacks registered by the nodes

ROS 2 offers different types of executors:

- **`SingleThreadedExecutor`**: processes callbacks sequentially, suitable for deterministic behavior
- **`MultiThreadedExecutor`**: allows callbacks to be processed in parallel, enabling concurrency across nodes and their internal timers

All registered nodes are added to the executor, which then handles all event-driven execution

This means the `tick_tock(...)` based tree ticking is handled just like any other timer based ROS callback through the executor’s queue

## Entry Point

A conventional Python entry point is used to initialize the `BehaviorTreeApp` and thereby launch the full ROS 2 system.

```python
from app import BehaviorTreeApp

def main():
    app = BehaviorTreeApp()
    app.run()

if __name__ == "__main__":
    main()
```

## Usage of  ament_cmake

As mentioned before, in ROS 2, packages can be built using either `ament_python` or `ament_cmake` as their build system

- **`ament_python`** is used for building _pure_ Python packages. It is suitable when the project consists only of Python scripts and does **not** define any custom ROS interfaces such as `.msg` or `.srv` files. It is limited in scope and typically used for lightweight tools, helper nodes, or prototypes

- **`ament_cmake`**, on the other hand, is the standard build system for ROS 2. It supports both **C++ and Python code**, and allows the generation and usage of custom message types, service definitions (`.srv`), and action definitions (`.action`). This makes it the **preferred choice for larger, more complex, and research-oriented robotics projects** that involve a wide variety of ROS features

### Migrating a package from ament_python to ament_cmake

Migrating an existing `ament_python` based package to `ament_cmake` is straightforward and can be done in a few simple steps:

1. **Delete** or deactivate `setup.cfg`
2. **Replace** `setup.py` with a `CMakeLists.txt` file that defines the Python installation and other build instructions
3. **Edit** `package.xml`:
	- Replace `<build_type>ament_python</build_type>` with `<build_type>ament_cmake</build_type>`
4. Optionally, add support for `.srv`, `.msg`, or `.action` files using `rosidl_generate_interfaces()` in `CMakeLists.txt`

No changes to the Python source code are required. Imports such as `py_trees`, `py_trees_ros`, and internal module structure remain untouched

### Example conversion 

#### `setup.py` (before migration)

```python
from setuptools import find_packages, setup

package_name = 'bipedal_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    # Install metadata and ROS-specific resources
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marvin',
    maintainer_email='marvin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    # Define executable Python nodes as console scripts
    entry_points={
        'console_scripts': [
            'imu_sensor = bipedal_robot_pkg.imu_sensor_node:main',
            'laser_sensor = bipedal_robot_pkg.laser_sensor_node:main',
            'battery_monitor = bipedal_robot_pkg.battery_monitor_node:main',
        ],
    },
)
```

**Note:** Every console script must also include the following _shebang_ at the very top of the Python file:

```python
#!/usr/bin/env python3
```

**Ensuring your Python console scripts are executable**

To ensure that Python based console scripts are executable after installation, the proper file permissions must be set **manually**

> This step is **mandatory** otherwise `ros2 run your_package your_script.py` may fail due to missing executable rights

#### Option 1 – If Git is used

To set and preserve executable rights in Git (so others don't need to do it manually):

-  Open a terminal and navigate to the workspace source:

```bash
cd ~/ros2_ws/src/bipedal_robot_pkg
```

-  Run:

```bash
git update-index --chmod=+x bipedal_robot_pkg/ros_nodes/sensors/*.py
```

*In this particular case the folder sensors contains all console scripts*

This marks the scripts as executable and Git will track this permission. Everyone who clones the repo will have the correct flags automatically after `colcon build`

#### Option 2 – If no Git is used

You can set the permissions manually (local only):

- Open a terminal and navigate to the workspace source:

```bash
cd ~/ros2_ws/src/bipedal_robot_pkg
```

- Make your sensor scripts executable:

```bash
chmod +x bipedal_robot_pkg/ros_nodes/sensors/*.py
```

This only applies **on the own machine**, so others would have to do the same

The line `USE_SOURCE_PERMISSIONS` in the `CMakeLists.txt` ensures these flags are copied **into the install directory**, but **it does not set them**. That's why this setup is needed once at the source level.

### Final `CMakeLists.txt` (after migration)

```CMake
cmake_minimum_required(VERSION 3.8)
project(bipedal_robot_pkg)

# Find required ROS and Python dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(py_trees REQUIRED)
find_package(py_trees_ros REQUIRED)

# Install Python package directory
install(
  DIRECTORY bipedal_robot_pkg
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install launch files (if any)
install(
  DIRECTORY bipedal_robot_pkg/behaviour_tree/launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install resource file for ROS package index
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install package.xml (mandatory for ROS2)
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install console scripts by installing Python package
ament_python_install_package(${PROJECT_NAME})

# Manually define console script executables
# Note: each console script must include the shebang '#!/usr/bin/env python3'
ament_python_install_modules()
install(PROGRAMS
  bipedal_robot_pkg/ros_nodes/sensors/imu_sensor_node.py
  bipedal_robot_pkg/ros_nodes/sensors/laser_sensor_node.py
  bipedal_robot_pkg/ros_nodes/sensors/battery_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Register Python executables with ROS environment
ament_environment_hooks(
  "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY}"
)

ament_package()
```

## Setting Conditions through Console via ROS2 Services

In ROS2, a **Service** represents a synchronous communication interface that can be called at any time, including from the console. Thanks to ROS2's multithreaded executor, such services are particularly suitable for **using external commands or triggers**, e.g. setting behavior tree conditions dynamically

It is considered **best practice** to implement a **dedicated service node** that exposes a **single service interface** for modifying internal behavior parameters such as blackboard conditions.

**Services rely on using ament_cmake**

### ROS2 Service Setup

A ROS2 Service mechanism consists of the following components:

1. **`SetCondition.srv`**  
    → *Defines the structure of the service request and response messages.*

2. **`ConditionServiceNode.py`**  
    → *Implements the server logic which modifies the behavior tree's blackboard on service request.*

These elements must be **explicitly registered** within the ROS2 package configuration:

- `package.xml`
- `CMakeLists.txt`

**SetCondition.srv**

**Location**: In the root of the ROS package, not within the Python submodule.

	-> `your_ros_pkg/srv/SetCondition.srv`

```srv
string key     # Name of the condition, e.g. "must_walk"
bool value     # Desired boolean value
---
bool success   # Whether the operation was successful
```

This service definition enables clients to dynamically set arbitrary keys on the PyTrees blackboard

**ConditionServiceNode.py**

```python
import rclpy
from rclpy.node import Node
from your_ros_pkg.srv import SetCondition  # Import the .srv definition

import py_trees


class ConditionServiceNode(Node):
    """
    A ROS2 service node that enables runtime modification of behavior tree conditions.

    It exposes a service `/set_condition`, which allows setting boolean flags 
    on the blackboard. These flags can be read by behavior tree condition nodes 
    to dynamically adapt robot behavior.
    """

    def __init__(self):
        super().__init__("condition_service_node")

        # Initialize blackboard write access
        self._blackboard = py_trees.blackboard.Client(name="ConditionServiceNode")
        self._blackboard.register_key("must_walk", access=py_trees.common.Access.WRITE)
        # Additional keys can be registered here as needed

        # Create ROS2 service
        self._srv = self.create_service(
            SetCondition,            # Auto-generated class from SetCondition.srv
            "set_condition",         # Service name
            self.set_condition_callback
        )

    def set_condition_callback(self, request, response):
        """
        Callback triggered when a client calls the `/set_condition` service.
        Sets a blackboard key to a specified boolean value.

        Args:
            request (SetCondition.Request): Contains 'key' (blackboard key name) and 'value' (boolean).
            response (SetCondition.Response): Will contain 'success' status after attempting the update.

        Returns:
            SetCondition.Response: Response indicating whether the operation was successful.
        """
        try:
            setattr(self._blackboard, request.key, request.value)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to set condition: {e}")
            response.success = False

        return response
```

### Runtime Usage from Console

```bash
ros2 service call /set_condition your_ros_pkg/srv/SetCondition "{key: 'must_walk', value: true}"
```

This call sets the "must_walk" key on the PyTrees blackboard. Any condition node relying on this key (such as `mustWalk`) will react accordingly during the next behavior tree tick

### Required Package Configuration

**CMakeLists.txt**

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetCondition.srv"
)
```

**package.xml**

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
```

# Final Words

This post evolves as I evolve. I will continuously refine and expand it as I deepen my understanding. Feedback and suggestions are always welcome!

---

#  Change Log

| Version | Date       | Changes                                                                                                                                                              |
| ------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1.1.5   | 2025-08-01 | Added Installing py_trees_ros for ROS 2 section                                                                                                                      |
| 1.1.4   | 2025-07-31 | Added Setting Conditions via Console section                                                                                                                         |
| 1.1.3   | 2025-07-29 | - Corrected error in Creating a New Package section<br>- Added Usage of  ament_cmake section                                                                         |
| 1.1.2   | 2025-07-28 | - Added Setting Conditions through Console via ROS2 Services section                                                                                                 |
| 1.1.1   | 2025-07-27 | - Updated Using YAML Section with new workflow                                                                                                                       |
| 1.1.0   | 2025-07-21 | - Added Application Class Based Initialization of ROS2 Systems section<br>- Added Entry Point section                                                                |
| 1.0.3   | 2025-07-20 | - Added Tree Factory Setup in ROS2 section                                                                                                                           |
| 1.0.2   | 2025-07-19 | - Added py_trees Action Node Setup section<br>- Added Core Integration Principles section<br>- Added Py_trees and ROS2 Workflow section<br>- Added Table of Contents |
| 1.0.1   | 2025-07-17 | - Added py_trees Condition Node setup                                                                                                                                |
| 1.0.0   | 2025-07-15 | - Added py_trees integration chapter                                                                                                                                 |
