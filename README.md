# PyTrees Copilot: Behavior Tree Runtime Verification 


This repository contains a pipeline to generate runtime monitors connected to behavior trees. 

The monitors are generated using [NASA Copilot](https://copilot-language.github.io/), a runtime verification framework built in Haskell Functional Programming Language. NASA Copilot allows for the generation of runtime monitors in C99 that can then be connected via ROS to Python applications. 

### Motivation  
There is an increasing number of behavior trees (BTs) being used in cyber-physical systems for their interpretability and modularity. These systems include safety and liveness specifications that need to be satisfied at runtime and compatible with BT execution. Therefore, we developed a pipeline to efficiently generate runtime verification monitors and integrate into behavior tree execution to estabilish assurance guarantees.



## Heater Example

### Preliminaries

Begin by opening and building the Docker using VSCode. 

* Open VSCode and Open Folder for this workspace. 
* Install Dev Containers Extension. 
* `CTRL+SHIFT+P` then search and click `Dev Containers: Rebuild and Repopen in Container`

Once the Docker Container has been opened and built in VSCode there will be a directory architecture of this type:

```
ros2_ws
    src
        .devcontainer
        example_heater_pkg
        heater_copilot
        file_generation
        generate_files.sh
        build_pkg.sh
        launch.sh
```

### Heater Monitor

The Heater monitor is a simple example with a temperature that monotonically increases. If the heat is too low (<18) the `heat on` trigger will activate. If the heat is too high (>21) the `heat off` trigger will activate.

The monitor is located in `src/heater_copilot`  
`cd src/heater_copilot` and open the Haskell file `Heater.hs`

```haskell
ctemp = extern "temperature" Nothing
```
An external variable `temperature` that will be provided by the application to the monitor.

```haskell
spec = do
  -- Triggers that fire when the ctemp is too low or too high,
  -- pass the current ctemp as an argument.
  trigger "heaton"  (ctemp < 18.0) [arg ctemp]
  trigger "heatoff" (ctemp > 21.0) [arg ctemp]
```
Definition of specifications to trigger when boolean condition is true.

```haskell
-- Compile the spec
main = reify spec >>= compile "monitor"
```

**Note**: Ensure you include `"monitor"` as your compilation target as this will be the name of the C99 files generated that will be included in the template below.

### Generate ROS2 Template

The mechanism we use to communicate between BTs and Monitors is ROS2. We developed a templating library to generate a ROS2 template to interact between the monitors defined above and a BT node created with the [PyTrees ROS](https://github.com/splintered-reality/py_trees_ros) Framework.

To create a template for our heater application follow these steps:

```bash
cd src
chmod +x generate_files.sh
./generate_files.sh -p heater_pkg -m heater_copilot/Heater.hs
```

You can specify any package name and the path to your Haskell Copilot Monitor
```bash
./generate_files.sh -p <package_name> -m <Haskell Copilot Monitor Path>
```

Generated Structure:
```
<package_name>
    launch
        tree.launch.py -- Launch Monitor and BT Node
    <package_name>
        __init__.py
        bt_node.py -- Contains Template BT Structure
        task_publish_var.py -- BT Behavior to publish extern (i.e. temperature)
        task_update_vars.py -- Template BT Behavior to update externs
    src
        monitor_node.cpp -- CPP Node to receive externs and publish monitor triggers
        monitor_types.h -- Autogen from Heater.hs
        monitor.c -- Autogen from Heater.hs
        monitor.h -- Autogen from Heater.hs
    include -- Place CPP Includes Here
```

### Build ROS2 Package

Next we build our package for execution  
First navigate back to the root, then execute the script.

```bash
cd ..
chmod +x src/build_pkg.sh
./src/build_pkg.sh -p heater_pkg
```

You can specify your package name instead.
```bash
./src/build_pkg.sh -p <package_name>
```

### Implement & Launch

The template is now compiled and linked, but we must update the file `src/<package_name>/<package_name>/task_update_vars.py` to increase the temperature at each tick (i.e. BT Execution Cycle) to achieve our intended behavior. 

You can find the full implementation of `task_update_vars.py` in `src/example_heater_pkg/example_heater_pkg/task_update_vars.py`

**Note**: This file can be modified or even replaced with the intended behavior for your system. `bt_node.py` can also be freely modified with new behaviors or structure. This is merely a starting point and not the final design for the BT. 

Finally, we launch our script to view the behavior tree execute and the value of temperature at each tick.

```bash
./src/launch.sh -p heater_pkg
```

You can specify your package name instead.
```bash
./src/launch.sh -p <package_name>
```

Example Output:

```
[bt_node.py-2] [ INFO] Publish temperature  : publishing: 6.0
[bt_node.py-2] 
[bt_node.py-2] /_/ BT Node [✕]
[bt_node.py-2]     {-} Topics to Blackboard [✓]
[bt_node.py-2]         --> heaton Subscriber [✓]
[bt_node.py-2]         --> heatoff Subscriber [✓]
[bt_node.py-2]     --> is_heaton? [✓] -- 'heaton' comparison succeeded [v: True][e: True]
[bt_node.py-2]     --> is_heatoff? [✕] -- 'heatoff' comparison failed [v: False][e: True]
[bt_node.py-2]     {-} Tasks [✓]
[bt_node.py-2]         --> Update Temperature [✓]
[bt_node.py-2]         --> Publish temperature [✓] -- published
```
