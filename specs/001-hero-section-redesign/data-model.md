# Data Model: Futuristic Hero Section and Logo Redesign

## Overview
Data model for the hero section redesign and custom logo implementation.

## Entities

### Hero Section Component
- **Type**: React Component
- **Properties**:
  - title: string ("Physical AI & Humanoid Robotics")
  - subtitle: string ("The Complete AI-Native Textbook")
  - tagline: string ("Master ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action Systems")
  - ctaText: string ("Start Reading →")
  - ctaLink: string ("/docs/intro")
  - backgroundColor: string ("#0f172a" or "#111")
  - accentColor: string ("#00d4ff")

### Logo Asset
- **Type**: Image Asset
- **Properties**:
  - path: string ("static/img/logo.png")
  - format: string ("PNG")
  - dimensions: string ("Recommended 40x40px or 64x64px")
  - background: string ("Transparent")
  - design: string ("Humanoid robot silhouette with glowing cyan neural circuits, encircled by metallic silver ring")

## UI Elements

### Typography Elements
- **Heading**: Large, bold, prominent display
- **Subtitle**: Medium size, supporting information
- **Tagline**: Smaller text, descriptive content
- **Button Text**: Clear, actionable language

### Layout Elements
- **Container**: Full-width, centered content area
- **Spacing**: Consistent vertical rhythm
- **Alignment**: Centered on desktop, left-aligned on mobile (as needed)

## Configuration Elements

### Docusaurus Config Updates
- **Navbar Logo**: Path to custom logo image
- **Favicon**: Updated to match branding
- **Site Title**: Updated to "Physical AI & Humanoid Robotics"
- **Tagline**: Updated to "The Complete Textbook for the Future of Robotics"