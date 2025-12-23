import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        {
          type: 'category',
          label: 'ROS 2 Basics',
          items: [
            'ros2-basics/index',
            'ros2-basics/dds-concepts',
            'ros2-basics/why-ros2-for-humanoids',
          ],
        },
        {
          type: 'category',
          label: 'Communication Model',
          items: [
            'communication-model/index',
            'communication-model/nodes-topics-services',
            'communication-model/rclpy-examples',
          ],
        },
        {
          type: 'category',
          label: 'Robot Structure',
          items: [
            'robot-structure/index',
            'robot-structure/urdf-basics',
            'robot-structure/python-ros-integration',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Advanced Digital Twin Integration (Gazebo & Unity)',
      items: [
        'advanced-digital-twin/index',
        {
          type: 'category',
          label: 'Advanced Physics Simulation',
          items: [
            'advanced-digital-twin/advanced-physics-simulation/index',
            'advanced-digital-twin/advanced-physics-simulation/compliant-contact-models',
            'advanced-digital-twin/advanced-physics-simulation/friction-models',
            'advanced-digital-twin/advanced-physics-simulation/multi-body-dynamics',
            'advanced-digital-twin/advanced-physics-simulation/physics-unity-integration',
            'advanced-digital-twin/advanced-physics-simulation/advanced-physics-examples',
          ],
        },
        {
          type: 'category',
          label: 'Multi-Platform Synchronization',
          items: [
            'advanced-digital-twin/multi-platform-synchronization/index',
            'advanced-digital-twin/multi-platform-synchronization/network-communication',
            'advanced-digital-twin/multi-platform-synchronization/time-synchronization',
            'advanced-digital-twin/multi-platform-synchronization/state-synchronization',
            'advanced-digital-twin/multi-platform-synchronization/performance-optimization',
            'advanced-digital-twin/multi-platform-synchronization/synchronization-examples',
            'advanced-digital-twin/multi-platform-synchronization/troubleshooting-guides',
          ],
        },
        {
          type: 'category',
          label: 'Advanced Sensor Fusion',
          items: [
            'advanced-digital-twin/advanced-sensor-fusion/index',
            'advanced-digital-twin/advanced-sensor-fusion/multi-sensor-integration',
            'advanced-digital-twin/advanced-sensor-fusion/kalman-filtering',
            'advanced-digital-twin/advanced-sensor-fusion/probabilistic-sensor-models',
            'advanced-digital-twin/advanced-sensor-fusion/validation-techniques',
            'advanced-digital-twin/advanced-sensor-fusion/sensor-fusion-examples',
            'advanced-digital-twin/advanced-sensor-fusion/best-practices',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'isaac-ai-brain/index',
        {
          type: 'category',
          label: 'NVIDIA Isaac Overview',
          items: [
            'isaac-ai-brain/nvidia-isaac-overview/index',
            'isaac-ai-brain/nvidia-isaac-overview/isaac-sim-vs-ros',
            'isaac-ai-brain/nvidia-isaac-overview/synthetic-data',
          ],
        },
        {
          type: 'category',
          label: 'Perception and Navigation',
          items: [
            'isaac-ai-brain/perception-navigation/index',
            'isaac-ai-brain/perception-navigation/vslam-concepts',
            'isaac-ai-brain/perception-navigation/sensor-pipelines',
          ],
        },
        {
          type: 'category',
          label: 'Training and Readiness',
          items: [
            'isaac-ai-brain/training-readiness/index',
            'isaac-ai-brain/training-readiness/path-planning',
            'isaac-ai-brain/training-readiness/sim-to-real',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'vla-integration/index',
        {
          type: 'category',
          label: 'VLA System Overview',
          items: [
            'vla-integration/vla-system-overview/index',
            'vla-integration/vla-system-overview/vla-architecture',
            'vla-integration/vla-system-overview/role-in-humanoid-autonomy',
          ],
        },
        {
          type: 'category',
          label: 'Language to Intent',
          items: [
            'vla-integration/language-to-intent/index',
            'vla-integration/language-to-intent/voice-to-text',
            'vla-integration/language-to-intent/llm-processing',
          ],
        },
        {
          type: 'category',
          label: 'Planning to Action',
          items: [
            'vla-integration/planning-to-action/index',
            'vla-integration/planning-to-action/intent-to-actions',
            'vla-integration/planning-to-action/humanoid-flow',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Capstone Project',
      items: [
        'capstone-project/index',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
