import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Book sidebar configuration for Physical AI & Humanoid Robotics
 *
 * Structure:
 * - Introduction
 * - Part 1: The Robotic Nervous System (ROS 2)
 *   - Chapter 1: ROS 2 Nodes, Topics, and Services (7 lessons)
 *   - Chapter 2: Bridging Python Agents to ROS Controllers (7 lessons)
 *   - Chapter 3: Understanding URDF for Humanoids (8 lessons)
 *
 * Note: Docusaurus strips number prefixes from folder names when generating IDs
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Part 1: The Robotic Nervous System',
      link: {
        type: 'doc',
        id: 'Part-1-ROS2-Foundation/index',
      },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Nodes, Topics, and Services',
          link: {
            type: 'doc',
            id: 'Part-1-ROS2-Foundation/ros2-nodes-topics-services/index',
          },
          items: [
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/introduction',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/understanding-nodes',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/topics-and-messages',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/publishers-and-subscribers',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/services-and-clients',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/quality-of-service',
            'Part-1-ROS2-Foundation/ros2-nodes-topics-services/capstone-integration',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Bridging Python Agents to ROS Controllers',
          link: {
            type: 'doc',
            id: 'Part-1-ROS2-Foundation/rclpy-python-development/index',
          },
          items: [
            'Part-1-ROS2-Foundation/rclpy-python-development/package-structure',
            'Part-1-ROS2-Foundation/rclpy-python-development/async-callbacks',
            'Part-1-ROS2-Foundation/rclpy-python-development/action-clients',
            'Part-1-ROS2-Foundation/rclpy-python-development/custom-messages',
            'Part-1-ROS2-Foundation/rclpy-python-development/executors-concurrency',
            'Part-1-ROS2-Foundation/rclpy-python-development/reusable-patterns',
            'Part-1-ROS2-Foundation/rclpy-python-development/capstone-python-agent',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Understanding URDF for Humanoids',
          link: {
            type: 'doc',
            id: 'Part-1-ROS2-Foundation/urdf-humanoid-modeling/index',
          },
          items: [
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/urdf-fundamentals',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/collision-inertial',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/joints',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/inertia-calculations',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/transform-frames',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/articulated-arm',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/xacro-patterns',
            'Part-1-ROS2-Foundation/urdf-humanoid-modeling/capstone-humanoid',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Part 2: The Digital Twin',
      link: {
        type: 'doc',
        id: 'Part-2-Digital-Twin/index',
      },
      items: [
        {
          type: 'category',
          label: 'Chapter 4: Simulating Physics in Gazebo',
          link: {
            type: 'doc',
            id: 'Part-2-Digital-Twin/gazebo-physics/index',
          },
          items: [
            'Part-2-Digital-Twin/gazebo-physics/gazebo-architecture',
            'Part-2-Digital-Twin/gazebo-physics/world-files',
            'Part-2-Digital-Twin/gazebo-physics/model-spawning',
            'Part-2-Digital-Twin/gazebo-physics/physics-tuning',
            'Part-2-Digital-Twin/gazebo-physics/collision-detection',
            'Part-2-Digital-Twin/gazebo-physics/joint-control',
            'Part-2-Digital-Twin/gazebo-physics/debugging-optimization',
            'Part-2-Digital-Twin/gazebo-physics/capstone-humanoid-balance',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: High-Fidelity Rendering and HRI in Unity',
          link: {
            type: 'doc',
            id: 'Part-2-Digital-Twin/unity-hri/index',
          },
          items: [
            'Part-2-Digital-Twin/unity-hri/unity-ros-bridge',
            'Part-2-Digital-Twin/unity-hri/urdf-import',
            'Part-2-Digital-Twin/unity-hri/environment-design',
            'Part-2-Digital-Twin/unity-hri/human-avatars',
            'Part-2-Digital-Twin/unity-hri/interaction-scripting',
            'Part-2-Digital-Twin/unity-hri/ros-integration',
            'Part-2-Digital-Twin/unity-hri/scene-management',
            'Part-2-Digital-Twin/unity-hri/capstone-hri-demo',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 6: Simulating Sensors',
          link: {
            type: 'doc',
            id: 'Part-2-Digital-Twin/sensors-simulation/index',
          },
          items: [
            'Part-2-Digital-Twin/sensors-simulation/sensor-architecture',
            'Part-2-Digital-Twin/sensors-simulation/urdf-sensors',
            'Part-2-Digital-Twin/sensors-simulation/lidar-simulation',
            'Part-2-Digital-Twin/sensors-simulation/depth-camera',
            'Part-2-Digital-Twin/sensors-simulation/imu-sensor',
            'Part-2-Digital-Twin/sensors-simulation/noise-models',
            'Part-2-Digital-Twin/sensors-simulation/rviz-visualization',
            'Part-2-Digital-Twin/sensors-simulation/sensor-processing',
            'Part-2-Digital-Twin/sensors-simulation/capstone-sensor-suite',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
