A ROS2-Gazebo scenario generator for evaluating social navigation algorithms utilizing LLMs and [Hunavsim](https://github.com/robotics-upo/hunav_sim)
![](imgs/pipeline.png)
![](imgs/sample_generated_scenarios.jpeg)

## Installation Instructions

- This package is tested on ROS2 Humble, Gazebo v11.10.2 with python 3.9. Currently, we support querying any GPT-based model.
- Install the required python packages from requirements.txt
- build and install the included hunav_sim and hunav_gazebo_wrapper ROS2 in your colcon workspace and source the setup.bash file

SocRATES consists of 3 submodules:
    - A scenario generation module that queries LLMs in a structured manner to generate parameters and files for simulating a scenario
    - Modified version of Hunavsim (included in this repo)
    - Modified version of the Hunavsim wrapper for gazebo
    - For the 2 ROS2 packages, clone the submodules into your catkin workspace and build with the ROS2 ``colcon`` tool

## Generating your own scenarios

You can generate a scenario in 3 steps:

- (optional) Annotate your map file with a scene graph
- Generate components of the scenario (robot&human trajectories and human behaviors)
- Simulate the scenario in Gazebo

To make things simple, you can use the GUI with ``python3 socrates_gui.py`` where all these steps can be performed. Run it from within the SocRATES folder

### Map Annotation (requires map.yaml, map.png)

- We have provided sample maps in the ``locations`` folder for reference. This step is optional. If you just want to simulate scenarios in the warehouse environment, skip this step.
- To use your own environment, ensure gazebo can find the assets used in your environment
- Add a folder with the same name as your environment in the hunav_gazebo_wrapper/worlds folder. This folder should contain a folder called ``worlds`` and your .world file with the same name as the environment. Outside this worlds folder,  please include a ``camera_position.json`` file. These coordinates should be relative to the world origin in gazebo. This will enable generating a video automatically of the generated scenario.

```
{
    'x':0.0,
    'y':0.0,
    'z':0.0,
    'yaw':0.0
}
```

1. Update ``annotator/config.yaml`` with the scene graph schema for your map, and the paths to the image files. You can also tune the ``zoom_in`` parameter to comfortably annotate the image and the ``zoom_out`` parameter to save the annotated image with the corresponding zoom. The schema already filled in is for a warehouse map. Use easy to understand/self-explanatory edge and node names. For example, simple-edge or simple-node for un-noteworthy edges and nodes respectively and specific names (like blind corner,intersection etc.) for important characterizing locations in the map.
2. Annotate your map image with the annotator tool:
   - ``cd annotator``
   - `` python annotator.py``
   - Follow the instructions in the terminal to generate a map image annotated with a scene graph. An example is shown in the [warehouse annotated map](locations/small_warehouse/scene_graph/scene_graph.png). Double click to add nodes and click and drag between 2 nodes to create edges. Remember to keep the terminal open, where you will be asked to enter the type of node and edge depending on your map schema.

### Scenario Generation

1. Configure the ``inputs.yaml`` file and specify the parameters for generating the scenario. Remember to specify the paths for the ROS2 packages depending on your ROS2 workspace. The main inputs are (check out the examples in ``sample_inputs.md``):
   - ``context``: Define the social context in which the robot is performing its task
   - ``rough scenario``: Specify if you want to generate a specific scenario. More specific and well defined scenarios tend to work better (Default ``None``).

- If you want to reuse parts of a previously generated scenario like the scenario description, trajectory or the behaviors, set the corresponding variable ``load__component_to_load: true``. For example, to regenerate only the human behaviors and keep the scenario and trajectories previously generated, use:

```
load_scenario_response: true 
load_trajectory_response: true 
load_bt_response: false
```

2. Run the scenario generator with ``python main.py``.

- You will be asked for confirmations for the generated scenarios and trajectories.
- The scenario generator could fail (LLMs are imperfect.), the script will try to retry scenario generation multiple times, please rerun if there is a full failure.
- If the trajectories or behaviors fail to generate, the scenarios maybe regenerated. You can refer to the scene graph image to inspect the proposed paths for the human and the robot.

### Launching the Scenario in Gazebo

- In a separate terminal, source the environment with ROS2 installed.
  If you've set the paths correctly, the scenario generation script should modify the files in the ``hunav_gazebo_wrapper`` module directy to orchestrate the scenario.

1. Build your ROS2 workspace with:

```
cd catkin_ws #change to your corresponding workspace
colcon build --packages-select hunav_gazebo_wrapper hunav_sim 
source install/setup.bash
```

2. Launch scenario with ``ros2 launch hunav_gazebo_wrapper scenario.launch.py``

- Due to the many ways in which LLMs can go wrong despite error handling approaches, if there's a (non-ROS related) error in this command, please regenerate the scenario.
- The scenario starts in a pause state, giving you time to open any other analysis tools or nodes. Remember to unpause the gazebo simulation with Space-bar.

3. Control the robot:
   - To Teleop: Run the teleop node. E.g.: ``ros2 run hunav_gazebo_wrapper teleop_keyboard_twist.py`` to control the robot in another terminal.
   - Use Nav2 planners: ``ros2 launch hunav_gazebo_wrapper nav2_follow_waypoints.launch.py params_file:=<nav2_params_file>``.
   - Use your own planner: implement the ``navigate_waypoints`` function in the ``hunav_gazebo_wrapper/scripts/waypointfollow.py``

- Also note that we throttle the movements of the humans in order to increase the likelihood of the generated scenario to occur in the simulation (for example, we make the human wait at a previous waypoint if the robot has not yet reached the point where the human and the robot must meet for the scenario. )

3. The humans and the robot in our framework can communicate through gestures. Run the following script to observe when the robot/human make a gesture: ``ros2 run hunav_gazebo_wrapper gesture_listener.py``. Publish to the ``/robot_gesture`` topic to make gestures towards humans with: ``ros2 run rqt_publisher rqt_publisher``. The integer robot gestures have the following semantic meaning: [0(No gesture), 1("WAIT"), 2("PROCEED"),3("ACKNOWLEDGED"),4("EXCUSE ME")].

- You can finetune the scenario parameters by modifying the saved files in the ``responses`` directory and setting the ``load`` parameters (like ``load_scenario_response``) to true and then running ``python main.py``. This will not regenerate the scenario, but update the gazebo simulation.
