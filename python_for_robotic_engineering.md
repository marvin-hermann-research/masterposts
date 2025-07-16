#Python for Robotics Engineering and Cognitive Architectures

---

## Why Python?

Python is the structural backbone of most modern AI and robotics research. It’s readable, flexible, and perfectly suited for rapid prototyping. This post serves as my evolving knowledge base for all things Python that are relevant to robotic system engineering and scientific software architecture.

---

## Environment Setup

### Create a virtual environment

`python3 -m venv venv`

### Activate the environment (Linux/macOS)

`source venv/bin/activate`

> Always activate from the parent directory of the `/venv` folder.

### Python Class / Packages Location

In the venv's parent folder

---

## Dependency Management

### Generate `requirements.txt`

In the venv's parent folder
`pip freeze > requirements.txt`

### Install from `requirements.txt`

`pip install -r requirements.txt`

> This ensures full reproducibility across systems (e.g., Git clones).

---

## Clean Code Conventions (Pythonic Style Guide)

### Naming

### File Names

Use `snake_case.py`

### Class Names

Use `class PascalCase`

> Should be a noun

### Method Names

Use `def snake_case`

> Should describe the **purpose** of the method

### Variable Names

`temperature, sensor_id`

>Describe the **contents** precisely

### Visibility

There is no real `private, protected, public` in Python. Only conventional indicators through naming

```python
self.sensor_name      # public
self._sensor_name     # protected
self.__sensor_name    # private
```

> applies to variables, methods and classes

---

## Smart Property Design (No Getter/Setter usage)

Use `@property` decorators

```python
@property
def temperature(self): 
	return self._temperature
	
@temperature.setter
def temperature(self, value):
	self._temperature = value`

print(sensor.temperature) # Getter usage
sensor.temperature = 22.5 # Setter usage
```

---
## Dunder Methods

Special methods in Python that are automatically invoked by the interpreter during certain operations.  
They start and end with` "__"

They are not just naming conventions -> they define **language-level behavior**

## Examples:

- `__init__(self, ...)`: Constructor, called by instantiation
- `__str__(self)`: Defines the string shown when using `print(obj)`
- `__repr__(self)`: Defines how the object is represented in the shell/debugger
- `__len__(self)`, `__getitem__`, `__setitem__`: Enable `len(obj)`, `obj[i] = val`, etc.
- `__eq__`, `__lt__`, `__gt__`: Comparison operators like `==`, `<`, `>`

---


```python
class Sensor:
	def __init__(self, type):
		self._sensor_type = type
```

Inheritance in Python allows classes to extend or specialize behavior from a parent class. 
	-> should only be used when it is **semantically justified**

### Creating a Child Class
To inherit from a parent class, include its name in parentheses

```python
class TemperatureSensor(Sensor):
```

The child class now has access to all **public** and **protected** methods and attributes of the parent
These inherited methods can be used directly or **overridden** by redefining them in the child. Overriding means the parent’s method is replaced **but** the original can still be accessed via 
`super()`

### Using `super()` to Access the Parent Implementation

If a method is overridden, but you still want to **invoke the parent version**, use `super()`

```python
super().method_name()
```

This is particularly common in constructors when the child class extends the initialization logic of the parent

```python
class Sensor:
    def __init__(self, sensor_type):
        self._sensor_type = sensor_type

    def read(self):
        raise NotImplementedError("Must be implemented by subclass")


class TemperatureSensor(Sensor):
    def __init__(self, sensor_type, calibration):
        super().__init__(sensor_type)           # Call parent constructor
        self._calibration = calibration         # Extend with new attribute

    def read(self):
        # Custom implementation overriding the abstract parent method
        return f"{self._sensor_type} reading: {round(random.uniform(20.0, 30.0), 2)}°C"
```

---

## Importing and instantiating Classes

To use a class defined in another file (module), import it using Python's module system

```python
from my_module import MyClass

obj = MyClass(type)
```

This imports `MyClass` from the file `my_module.py`, and instantiates it by calling its constructor.
## Execution Entry

Python uses a conditional entry point that resembles Java's **main()** function, but it’s **not a function**, and it is written **at the top level of the file**, not inside a class

```python
if __name__ == "__main__":
	my_instance.run()
```

This conditional ensures that the code block only runs **when the file is executed directly**, and **not when it is imported** as a module

---

## Essential Built-In Functions

### File Handling with Auto-Close

The **with open(...) as ...** pattern ensures that a file is **automatically closed**, even in case of errors –> no need to manually call **close()**

```python
with open("file.txt", "a") as f:
    f.write("New log entry\n")
```

| Mode  | Purpose                          |
| ----- | -------------------------------- |
| `"r"` | Read (default)                   |
| `"w"` | Write (overwrite file)           |
| `"a"` | Append (add to file)             |
| `"x"` | Write only if file doesn't exist |
| `"b"` | Binary mode                      |
### Random Numbers

```python
import random
random.uniform(a, b)  # Returns a float between a and b
```

### Rounding Numbers

```python
round(float, n)  # Rounds a float to `n` decimal places.
```

### Sleep / Timing

Essential when ticking loops, simulating delays, or rate-limiting processes

```python 
 import time

time.sleep(1.5)  # Pause the program for 1.5 seconds

```

### Efficient String Building

Use **f-string**` to build dynamic messages in a readable and performant way
	
=> Favor `f""` formatting whenever variables are included. It's faster and clearer than traditional `%` formatting or `.format()` calls

```python
sensor_id = "TS-01"
temp = 24.7
unit = "°C"

msg = f"[{sensor_id}] Temperature: {temp:.1f}{unit}"
print(msg)
```

also usefull: `" ".join([...])` for list-based strings

---

### Flexible Function Parameters

Python offers two ways to pass an arbitrary number of arguments

#### `*args` — Positional Argument Collection

To use when expecting multiple values of the same kind

```python
def log_multiple(*messages):
	for m in messages:
		print(m)

log_multiple("Hi", "I'm", "cool")
```

#### `**kwargs` — Named Arguments (Dictionary)

To use when expecting various **named** values
=> use **.items()** to read them

```python
def configure_sensor(**settings):
	for key, value in settings.items():
	    print(f"{key} = {value}")
	         
configure_sensor(unit="Celsius", interval=5, active=True)
```

---

### Lambda Functions

Use for short, one-off functions passed as arguments

```python
square_and_add = lambda x, y: x * x + y print(square_and_add(3, 2))
```

Great for filtering, mapping, or sorting

```python
result = list(filter(lambda x: x > 10, data))
```

---

## Working with Lists

### List Comprehension

A readable way to transform lists

```python
squares = [x**2 for x in range(5)]
```

### List Window

```python
values = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
mid_index = len(values) // 2   # → 5 in this case

start_index = mid_index - 2    # includes index 3
end_index   = mid_index + 2    # excludes index 7

window = values[start_index : end_index]  # → [3, 4, 5, 6]
```

### `map()`

Applies a function to every element

```python
map(function,  liste)
```

### `filter()`

Filters out elements where the function returns `False`

```python
high_values = list(filter(lambda x: x > 10, [5, 12, 17, 3]))
```

---

## For-Loops, Range, Enumerate and Zip

### Repetition

```python
for i in range(3):
	print(i)
```

### Iterate Through a List

```python
for item in my_list:
	print(item)
```

### Check Whole List for Condition

```python
no_obstacle = all(distance >= 0.5 for distance in my_list)
```

### Index + Value: `enumerate()`

```python
for i, val in enumerate(my_list):
	print(i, val)
```

### Parallel Iteration: `zip()`

Iterates through 2 lists

```python
sensor_ids = ["TS-01", "TS-02"]
temperatures = [22.5, 27.5]

for sensor, temp in zip(sensor_ids, temperatures):
	print(f"{sensor}: {temp}°C")
```

---

## Exception Handling

Use `try/except` for anything that interacts with unpredictable systems: files, hardware, APIs, users

```python
try:
	risky_code()
except ValueError as e:
	print(f"Oops! Something went wrong: {e}")
```

---

## `__repr__()` — Debugging Output

When `print(obj)` yields a cryptic memory address e.g. **0x7f9a324e7ac0**, implement

```python
def __repr__(self):
	return f"Sensor(type={self._sensor_type})"
```

This defines how the object shows up in debug output or print statements

---

## PyTrees

Enable building of state maschines

```bash
pip install py_trees
```
### What Are PyTrees?

- **Blackboard**: Shared memory across nodes
- **Behavior Nodes**:
    - Action Nodes: Perform operations
    - Condition Nodes: Evaluate transitions
    - Composite Nodes: Control flow logic (`Selector`, `Sequence`, `Parallel`)

### Key Concepts and Conventions

- Nodes must be manually **ticked** to update => fires uptade function of each node
- Nodes are separated into individual files/modules
- A `TreeFactory` centralizes construction logic
- `main.py` acts as the runtime controller
- A Client system is used to access Blackboard data

### General organization

my_project/
├── nodes/
│   ├── condition_is_grounded.py
│   ├── condition_target_in_range.py
│   ├── action_move_forward.py
│   ├── action_cast_spell.py
├── tree_factory.py
├── sensor_writer.py
├── target_writer.py
├── main.py

### Condition Nodes &  Read/Write Clients

Condition nodes are responsible for **checking variables** that determine whether a behavior tree transition should be triggered

To achieve this, a **client-based blackboard system** is used to **create, read, and write** shared variables. 
These variables are **exclusively accessed via the tree architecture**, not through direct object references

Within this system:

- Condition nodes declare their clients with `Access.READ` → they **consume** values
- External components (e.g., sensors or trackers) declare clients with `Access.WRITE` → they **publish** values
- condition nodes **must have** an *update* function. This function gets called automatically every tree tick

``` python
import py_trees  # Import

class IsGrounded(py_trees.behaviour.Behaviour):  # Inherit from py_trees base behavior
    def __init__(self):
        super().__init__(name = "Is Grounded?")  # Define the node's name for logging and visualization

        # Create a blackboard client that will access shared variables
        self.blackboard = py_trees.blackboard.Client(name = "IsGroundedClient")

        # Register a variable on the blackboard in read-only mode
        # This variable is assumed to be written by another component (e.g., a sensor class)
        self.blackboard.register_key(
            key = "is_grounded", 
            access = py_trees.common.Access.READ
        )

    def update(self):
        # Condition nodes return either SUCCESS or FAILURE depending on the variable's state
        if self.blackboard.is_grounded:
            self.logger.debug("Robot is grounded")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("Robot is NOT grounded")
            return py_trees.common.Status.FAILURE
```

The **WRITE** classes interact with the blackboard by registering and updating shared state variables. These classes serve as data providers

Using the same blackboard key in both the writer and the condition node enables shared access to the same variable 

=>write access must be registered by the writer, and read access by the condition node

```python
import py_trees

class SensorProcessor:
    def __init__(self):
        # Instantiate a blackboard client with write access
        self.blackboard = py_trees.blackboard.Client(name = "SensorWriter")

        # Register the shared variable to be writable
        self.blackboard.register_key(
            key = "is_grounded", 
            access = py_trees.common.Access.WRITE
        )

    def update_grounded_status(self, sensor_value: bool):
        # Write the updated grounded status to the blackboard
        self.blackboard.is_grounded = sensor_value

class TargetTracker:
    def __init__(self):
        # Create a separate blackboard client for target-related data
        self.bb = py_trees.blackboard.Client(name = "TargetWriter")

        # Register the variable as writable
        self.bb.register_key(
            key = "target_in_range", 
            access = py_trees.common.Access.WRITE
        )

    def update_target_range(self, in_range: bool):
        # Update target visibility status
        self.bb.target_in_range = in_range
```

### Action Nodes

Action Nodes are responsible for **executing concrete behaviors** within the tree. They typically follow a **lifecycle of initialization, execution, and termination**, reporting their current status via the standard `py_trees` return codes

- `RUNNING` while the action is in progress
- `SUCCESS` when the action completes successfully
- `FAILURE` if the action cannot be carried out

Internally, these nodes usually act as interfaces that **trigger methods of other classes** to perform the actual logic

```python
import py_trees
import time

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self):
        # Define the name of this node in the tree
        super().__init__(name="Move Forward")
        self.started = False

    def initialise(self):
        # Called every time the node is entered (ticked for the first time)
        self.started = False

    def update(self):
        # If this is the first tick, start the action
        if not self.started:
            print("Starting forward motion...")
            self.started = True
            self.start_time = time.time()
            return py_trees.common.Status.RUNNING

        # If the action has been running for more than 2 seconds, consider it done
        if time.time() - self.start_time > 2.0:
            print("Forward motion complete.")
            return py_trees.common.Status.SUCCESS

        # Otherwise, the action is still running
        return py_trees.common.Status.RUNNING
```

### Factory Setup

The **Tree Factory** pattern abstracts the construction of complex behavior trees into a centralized module
This allows trees to be **declared once** and instantiated dynamically at runtime -> improving modularity, testability, and clarity of system design

Each factory method creates and returns a predefined subtree composed of condition and action nodes

```python
from py_trees.composites import Selector, Sequence, Parallel
from py_trees.common import ParallelPolicy

# Import custom nodes
from nodes.condition_is_grounded import IsGrounded
from nodes.action_move_forward import MoveForward
from nodes.condition_is_grounded import IsTargetInRange
from nodes.action_move_forward import ScanTarget

class TreeFactory:

    @staticmethod
    def create_locomotion_tree():
        # Selector runs children in order and returns on first SUCCESS
        root = Selector(name = "Locomotion")
        root.add_children([
            IsGrounded(),
            MoveForward()
        ])
        return root

    @staticmethod
    def create_ability_tree():
        # Sequence requires all children to return SUCCESS (in order)
        root = Sequence(name = "Abilities")
        root.add_children([
            IsTargetInRange(),
            ScanTarget()
        ])
        return root

    @staticmethod
    def create_root_parallel_tree():
        # Parallel node ticks all children simultaneously
        root = Parallel(
            name = "Root Layer",
            policy = ParallelPolicy.SuccessOnAll()  # All subtrees must return SUCCESS
        )
        root.add_children([
            TreeFactory.create_locomotion_tree(),
            TreeFactory.create_ability_tree()
        ])
        return root

```

### Tree Execution example

```python
import time
import py_trees

from tree_factory import TreeFactory        # Factory providing predefined subtrees
from sensor_writer import SensorProcessor   # Blackboard WRITE client for sensor state
from target_writer import TargetTracker     # Blackboard WRITE client for target state

def main():
    # Create the root tree which runs multiple subtrees in parallel
    root = TreeFactory.create_root_parallel_tree()
    tree = py_trees.trees.BehaviourTree(root)

    # Initialize writer classes responsible for updating the blackboard
    sensor = SensorProcessor()
    tracker = TargetTracker()

    # Set initial blackboard states
    sensor.update_grounded_status(True)
    tracker.update_target_range(False)

    print("=== Starting Tree Execution ===")
    for i in range(10):
        # Dynamically modify blackboard states during runtime
        if i == 5:
            sensor.update_grounded_status(False)
            tracker.update_target_range(True)

        print(f"\n--- Tick {i} ---")
        tree.tick()
        time.sleep(1.0)

if __name__ == "__main__":
    main()

```

---

## Execution Logic of Behavior Trees

Each tree node decides **what to do next**, based on structural rules and runtime conditions

### Root as the Primary Branching Point

In most setups, the **root node** is a `Selector`, which acts as the central decision-maker
It attempts each of its children **in order**, and selects the **first one** whose internal condition returns `SUCCESS`

RootSelector
├── Combat Mode
├── Charge Battery
├── Explore
└── Idle

> Only the **first successful path** is ticked. Remaining branches are skipped unless the current one fails in future ticks

This makes selectors ideal for building **priority-based fallback systems**

---
### Example: Combat Mode as a Sequence

Sequence("Combat Mode")
├── Condition: Enemy Detected
├── Selector
│   ├── Use Ranged
│   └── Use Melee

- If `Enemy Detected` returns `FAILURE`, the entire sequence halts
- If it returns `SUCCESS`, the inner selector ticks and evaluates its options:  
    It may choose `Use Ranged` or `Use Melee`, depending on availability or cooldowns

---

### What if Multiple States Are True Simultaneously?

In cases where multiple branches should execute **in parallel** a `Selector` is no longer sufficient

Instead, use a **Parallel Node**:

```python
root = py_trees.composites.Parallel(
    name="Root",
    policy=py_trees.common.ParallelPolicy.SuccessOnAll()
)
```

- All children of the parallel node are ticked **simultaneously**
- Each subtree evaluates independently, based on its own local conditions

> This mirrors a **layered control model**, like Unity’s behavior layers

---

## Final Words

This post evolves as I evolve. I will continuously refine and expand it as I deepen my understanding. Feedback and suggestions are always welcome!

---

## Change Log

**Last Updated: 25.07.15**  
- Added **clarification** on how to access blackboard clients as `Writers` when working with `py_trees`
- Added **installation note**: `py_trees` must be installed via `pip`
- Implemented a **condition check** for the loop labeled `"all"` to improve control flow logic
- Documented the **creation of a list window** 
- Added a **note on `update()` function naming** for custom `Condition` nodes, to clarify behavior within behavior trees
