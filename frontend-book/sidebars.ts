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
      label: 'The Robotic Nervous System (ROS 2)',
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
