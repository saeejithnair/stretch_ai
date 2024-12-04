# Overview of `stretch_ai` repository

This document provides a detailed explanation of the `stretch_ai` repository, recently released by [Stretch Robotics](https://hello-robot.com/stretch-ai) and [Chris Paxton](https://itcanthink.substack.com/p/introducing-stretch-ai). It breaks down the system pipeline into distinct stages, focusing on how different components contribute to perception, mapping, and action.

The Stretch_AI pipeline starts with data capture from two cameras: a Head Camera at 640x480 resolution and a Wrist Camera at 320x240 resolution. This data feeds into a mapping system that builds both 2D and 3D representations using Hector SLAM and ORB-SLAM3. The perception system supports multiple detection models including Detic, YOLO, YOLO-World, and OWL-ViT, along with various captioning models like Moonbeam, BLIP, and ViT-GPT2. SAM2 handles segmentation while a SigLIP encoder extracts key features from detected objects. The system maintains a memory of objects it has seen and can match them across different views.

The pipeline integrates all components through sensor fusion, combining camera feeds with robot position data. ArUco markers help align point clouds and register object locations in the environment, while DynaMem continuously updates the semantic map. The entire system uses a hierarchical YAML configuration system that allows easy switching between different detection modules, captioners, and encoders. This modular architecture enables the system to process multiple operations in parallel and respond to environmental changes in real-time, making it effective for tasks ranging from navigation to object manipulation.


# Table of Contents

1. [Overview of Stretch_AI Repository](#overview-of-stretch_ai-repository)
2. [Pipeline](#pipeline)
   - [Data Ingestion](#data-ingestion)
   - [Mapping: Building a 3D Understanding of the Environment](#mapping-building-a-3d-understanding-of-the-environment)
   - [Integration of Perception and Mapping](#integration-of-perception-and-mapping)
   - [Object Detection and Segmentation](#object-detection-and-segmentation)
   - [Visual Understanding and Scene Representation](#visual-understanding-and-scene-representation)
   - [Sensor Fusion](#sensor-fusion)
3. [System Capabilities](#system-capabilities)
4. [Data Collection with Dex Teleop](#data-collection-with-dex-teleop)
5. [Configuration Guide](#configuration-guide)
   - [Main Configuration Files](#main-configuration-files)
   - [Configuration Hierarchy](#configuration-hierarchy)
   - [Key Parameter Groups](#key-parameter-groups)
   - [Configuration Tips](#configuration-tips)
6. [End-to-End Pipeline Example](#end-to-end-pipeline-example)
   - [Entry Point and Initialization](#entry-point-and-initialization)
   - [Mapping and Exploration](#mapping-and-exploration)
   - [Object Detection and Manipulation](#object-detection-and-manipulation)
   - [Navigation and Manipulation](#navigation-and-manipulation)
   - [Key Components and Their Interactions](#key-components-and-their-interactions)
   - [Configuration Flow](#configuration-flow)
   - [Visualization and Debugging](#visualization-and-debugging)


# Pipeline
## Data Ingestion

This stage focuses on capturing input data from the robot's sensors.

**Input Format:**
- Two main cameras: **Head Camera** and **Wrist Camera**
  - Resolutions: 
    - RGB: 320x240 (Wrist), 640x480 (Head)
    - Depth Maps: Matching resolutions

**Data Storage and Transformation:**
- RGB and depth streams from each camera are captured and processed for downstream tasks, such as mapping and object detection
- Camera data is handled through ROS2 topics and integrated with the perception pipeline

**Implementation:**
- Camera data is processed through the perception module: `src/stretch/perception/`
- Key components:
  - Detection pipeline: `src/stretch/perception/detection/`
  - Visual encoders: `src/stretch/perception/encoders/`
  - Captioning systems: `src/stretch/perception/captioners/`

**Examples:**
```bash
# View all camera feeds through ROS2
ros2 run image_tools showimage /camera/color/image_raw
ros2 run image_tools showimage /camera/depth/image_raw
```

## Mapping: Building a 3D Understanding of the Environment

This stage creates both 2D and 3D maps for navigation and semantic understanding.

**Mapping Techniques:**
- **Hector SLAM**: 2D occupancy mapping for navigation
- **ORB-SLAM3**: Visual SLAM for 3D mapping

**Run Examples:**
```bash
# Hector SLAM
ros2 launch stretch_ros2_bridge startup_stretch_hector_slam.launch.py

# ORB-SLAM3
ros2 launch stretch_ros2_bridge startup_stretch_orbslam.launch.py
```

**Implementation:**
- Code: `src/stretch/mapping/`

## Integration of Perception and Mapping

3D maps are used for navigation and task execution.

**Types of Maps:**
- **Voxel Map**:
  - Used for navigation and collision avoidance
  - Input: Depth camera point clouds
  - Output: 3D occupancy grid
- **Instance Map**:
  - Used for object tracking and semantic understanding
  - Input: Detection/segmentation results + point clouds
  - Output: Object instances with positions and labels

**Dynamic Semantic Mapping (DynaMem):**
- Provides a live, continuously updated representation of the environment
- Stores object locations and semantic information for:
  1. Navigation
  2. Planning
  3. Interaction
- Links natural language commands to physical locations

**Implementation:**
- `src/stretch/mapping/dynamem`

## Object Detection and Segmentation

This stage extracts visual understanding from raw sensor data.

**Detection:**
- Method: **Detic** (from Facebook)
- Output: Bounding boxes and class labels

**Segmentation:**
- Method: **SAM2**
- Output: Instance masks

**Implementation:**
- Detection pipeline: `src/stretch/perception/detection/`
- Visual feature extraction: `src/stretch/perception/encoders/`
- Perception wrapper: `src/stretch/perception/wrapper.py`

## Visual Understanding and Scene Representation

The perception pipeline integrates detection with visual understanding for robust scene comprehension.

**Key Components:**
- **Object Recognition**: Uses configured detection module (Detic/YOLO/etc.)
- **Feature Extraction**: SigLIP encoder for object embeddings
- **Scene Understanding**: Combines detection and features for:
  - Object classification
  - Spatial relationships
  - Natural language grounding

**Memory and Matching:**
- Maintains instance memory of seen objects
- Uses feature matching for object persistence
- Configurable matching thresholds:
```yaml
instance_memory:
  matching:
    feature_match_threshold: 0.05  # Adjust based on needs
  min_instance_thickness: 0.01
  min_instance_vol: 1e-6
  max_instance_vol: 10.0
```

**Integration with Mapping:**
- Links detected objects to 3D positions
- Maintains object persistence across views
- Enables natural language queries about object locations

**Run Examples:**
```bash
# Test captioning models
python -m stretch.perception.captioners.moonbeam_captioner --image_path object.png
python -m stretch.perception.captioners.vit_gpt2_captioner --image_path object.png
python -m stretch.perception.captioners.blip_captioner --image_path object.png
```

## Sensor Fusion

This stage combines data from multiple sensors to create a unified representation of the environment.

**Methods:**
- Combines RGB-D data from head and wrist cameras with robot pose from odometry and SLAM
- Uses **ArUco marker detection** for:
  - Unifying point clouds in the world frame
  - Registering object detections with semantic maps

**Implementation:**
- Fusion code: `src/stretch/agent/robot_agent.py`
- ArUco integration: `src/stretch/perception/aruco/`

## System Capabilities

By integrating these components, the system enables:
1. **Natural Language Navigation**: E.g., "Go to the cup"
2. **Object Manipulation**: E.g., "Pick up the toy"
3. **Dynamic Environment Understanding**: Supports live updates without rescanning
4. **Continuous Operation**: Adapts to changes in real-time

## Data Collection with Dex Teleop

For data collection, the system uses a low-cost teleoperation framework, **Dex Teleop**.

**Key Features:**
- Supports teleoperation modes such as base control and arm manipulation
- Captures RGB-D data streams for dataset generation

**Implementation:**
- Code: `src/stretch/app/dex_teleop/`

**Run Examples:**
```bash
# On robot
ros2 launch stretch_ros2_bridge server.launch.py

# On PC
python3 -m stretch.app.dex_teleop.ros2_leader --task-name default_task --teleop-mode base_x --save-images
```

# Configuration Guide

The Stretch_AI system uses a hierarchical configuration system with different YAML files for different purposes. Here's how to configure the system effectively:

## Main Configuration Files:

1. **dynav_config.yaml** (Dynamic Navigation):
   - Primary configuration for real-time operation
   - Contains basic settings for:
     ```yaml
     # Perception
     detection:
       module: "detic"  # Or "yolo", "yolo_world", "sam"
       category_map_file: "example_cat_map.json"
     
     # Mapping
     voxel_size: 0.1
     obs_min_height: 0.2
     obs_max_height: 1.5
     
     # Instance memory
     instance_memory:
       min_pixels_for_instance_view: 100
       feature_match_threshold: 0.05
     ```

2. **default_planner.yaml**:
   - More detailed configuration for planning and perception
   - Overrides dynav_config.yaml when using the planner
   - Includes:
     ```yaml
     # Perception settings
     encoder: "siglip"
     encoder_args:
       version: "so400m"
       feature_match_threshold: 0.1
     
     # Motion planning
     motion_planner:
       algorithm: "rrt_connect"  # ["rrt", "rrt_connect", "a_star"]
       step_size: 0.05
       rotation_step_size: 0.1
     ```

3. **Category Map Files** (JSON):
   - `example_cat_map.json`: Default object categories
   - `pickup_cat_map.json`: Specific categories for manipulation
   - Define mappings between detection labels and robot actions

## Configuration Hierarchy:

1. **Base Settings**: `dynav_config.yaml`
   - Basic operation parameters
   - Real-time navigation settings
   - Simple perception setup

2. **Planning Override**: `default_planner.yaml`
   - More sophisticated settings
   - Overrides basic settings when planning
   - Detailed motion and perception parameters

3. **Task-Specific**: In `config/app/`
   - Specialized configs for specific applications
   - Override base settings for particular tasks

## Key Parameter Groups:

1. **Perception**:
   ```yaml
   detection:
     module: "detic"  # Core detection method
     yolo_world_model_size: "l"  # If using YOLO World
     confidence_threshold: 0.2
   encoder: "siglip"  # Feature extraction
   ```

2. **Mapping**:
   ```yaml
   voxel_size: 0.04  # Map resolution
   obs_min_height: 0.10
   obs_max_height: 1.8
   pad_obstacles: 3
   ```

3. **Motion Planning**:
   ```yaml
   motion_planner:
     algorithm: "rrt_connect"
     step_size: 0.05
     simplify_plans: true
   ```

4. **Instance Memory**:
   ```yaml
   instance_memory:
     min_instance_vol: 1e-6
     max_instance_vol: 10.0
     feature_match_threshold: 0.05
   ```

## Configuration Tips:

1. **Start with Defaults**:
   - Use `dynav_config.yaml` for basic setup
   - Only override what you need to change

2. **Task Tuning**:
   - Adjust detection thresholds based on task:
     - Lower for more detections
     - Higher for more confidence
   - Tune motion parameters for speed vs. precision

3. **Performance Optimization**:
   - Adjust voxel_size for map resolution vs. speed
   - Tune instance memory thresholds for reliability
   - Configure filters for point cloud quality

4. **Debugging**:
   - Enable visualization flags for debugging
   - Adjust logging and verbosity as needed
   - Use motion thresholds to diagnose issues


# End-to-End Pipeline Example

Here's a walkthrough of how the system operates, using the main `ai_pickup` demo as an example:

## Entry Point and Initialization
```bash
python -m stretch.app.ai_pickup
```

**Pipeline Stages:**
1. **Initialization**:
   - Loads configuration from `config/default_planner.yaml`
   - Initializes robot client and perception modules
   - Sets up Rerun visualization

2. **Perception Setup**:
   ```python
   semantic_sensor = create_semantic_sensor(
       parameters=parameters,
       device_id=device_id,
       verbose=verbose
   )
   agent = RobotAgent(robot, parameters, semantic_sensor)
   ```

## Mapping and Exploration
```bash
# Can also be run standalone
python -m stretch.app.mapping --explore-iter 10
```

**Pipeline Stages:**
1. **SLAM Initialization**:
   - Starts either Hector SLAM (2D) or ORB-SLAM3 (3D)
   - Initializes voxel map for environment representation

2. **Exploration Loop**:
   ```python
   agent.start(goal=object_to_find)
   # Internally:
   # 1. Captures sensor data
   # 2. Updates voxel map
   # 3. Plans next exploration point
   # 4. Moves robot
   ```

## Object Detection and Manipulation
```bash
# Can test standalone with:
python -m stretch.app.grasp_object --target_object "cup"
```

**Pipeline Stages:**
1. **Perception**:
   ```python
   # In perception/wrapper.py
   if self._detection_module == "detic":
       # Initialize Detic detector
   elif self._detection_module == "yolo":
       # Initialize YOLO detector
   # Process images and return detections
   ```

2. **Planning and Execution**:
   ```python
   # In agent/robot_agent.py
   agent.move_to_object(target_object)
   agent.grasp_object()
   ```

## Navigation and Manipulation
**Pipeline Stages:**
1. **Path Planning**:
   ```python
   # Uses configured planner (RRT/A*/etc)
   res = self.planner.plan(current, target)
   robot.execute_trajectory(res.trajectory)
   ```

2. **Manipulation**:
   - Switches to manipulation mode
   - Executes pre-trained skills
   - Returns to navigation mode

## Key Components and Their Interactions:

1. **Robot Agent** (`agent/robot_agent.py`):
   - Main interface coordinating all components
   - Manages state transitions
   - Coordinates planning and execution

2. **Perception** (`perception/wrapper.py`):
   - Handles all visual processing
   - Manages different detection models
   - Provides object detection and segmentation

3. **Mapping** (`mapping/voxel.py`):
   - Maintains environment representation
   - Handles SLAM integration
   - Provides navigation space

4. **Motion Planning** (`motion/algo/`):
   - Implements different planning algorithms
   - Handles trajectory generation
   - Manages collision avoidance

## Configuration Flow:
1. Base config: `dynav_config.yaml`
2. Planning override: `default_planner.yaml`
3. Task-specific: `config/app/.yaml`

## Visualization and Debugging:
```bash
# Show intermediate maps
python -m stretch.app.mapping --show-intermediate-maps

# Visualize specific components
python -m stretch.app.read_map -i map.pkl --show-instances
```