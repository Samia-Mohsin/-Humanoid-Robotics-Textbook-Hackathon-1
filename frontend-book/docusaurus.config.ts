import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'The Complete Textbook for the Future of Robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://samia.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'samia', // Usually your GitHub org/user name.
  projectName: '-Humanoid-Robotics-Textbook-Hackathon-1', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/samia/-Humanoid-Robotics-Textbook-Hackathon-1/edit/main/frontend-book/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/textbook-social-card.png',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/samia/-Humanoid-Robotics-Textbook-Hackathon-1',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {
              label: 'Modules',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'ROS Answers',
              href: 'https://answers.ros.org/',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/robotics',
            },
            {
              label: 'ROS Discourse',
              href: 'https://discourse.ros.org/',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/samia/-Humanoid-Robotics-Textbook-Hackathon-1',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: [
        'python',      // Essential for ROS 2 Python nodes and scripts
        'bash',        // For ROS 2 command-line tools and shell scripts
        'yaml',        // For ROS 2 configuration files and launch files
        'json',        // For configuration and data exchange
        'cpp',         // For ROS 2 C++ nodes and core development
        'cmake',       // For CMakeLists.txt build configuration files
        'docker',      // For Dockerfiles and containerization examples
        'markdown',    // For documentation and README files
        'regex',       // For pattern matching and text processing examples
        'diff',        // For showing code differences and patches
        'ini',         // For configuration files (like .cfg files)
        'rust',        // For robotics applications using Rust
        'java',        // For Android-based robotics interfaces
        'typescript',  // For web-based robotics tools and interfaces
        'protobuf',    // For ROS 2 message definitions and interfaces
        'systemd',     // For systemd service files in robotics systems
      ],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
