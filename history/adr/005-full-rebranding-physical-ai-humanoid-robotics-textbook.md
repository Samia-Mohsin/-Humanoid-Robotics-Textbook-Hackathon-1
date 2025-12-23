# ADR-005: Full Rebranding and Navigation Alignment to "Physical AI & Humanoid Robotics Textbook"

**Status**: Accepted
**Date**: 2025-12-21

## Context

- Current title "ROS 2 for Humanoid Robotics" is too narrow – textbook covers Physical AI, ROS 2, Digital Twins, NVIDIA Isaac, and VLA systems
- Hackathon requires the submission to represent the complete "Physical AI & Humanoid Robotics" curriculum
- Users expect clear, broad branding from first glance
- Project: Physical AI & Humanoid Robotics Textbook Hackathon (Panaversity/PIAIC/GIAIC)
- Current State: Navbar title currently shows "ROS 2 for Humanoid Robotics", Landing page and branding partially focused on ROS 2 only, Sidebar has modules but top-level naming and hierarchy needs alignment with full textbook scope

## Decision

Implement comprehensive rebranding:

- Update docusaurus.config.ts: title, tagline, navbar logo text to "Physical AI & Humanoid Robotics"
- Update landing page (src/pages/index.tsx): main heading to "Physical AI & Humanoid Robotics: The Complete Textbook"
- Restructure sidebar: Create top-level "Textbook" category containing the 4 modules as collapsible sub-sections
- Module labels:
  - Module 1: The Robotic Nervous System (ROS 2)
  - Module 2: Advanced Digital Twin Integration (Gazebo & Unity)
  - Module 3: The AI-Robot Brain (NVIDIA Isaac™)
  - Module 4: Vision-Language-Action (VLA)
- Optional: Add subtle tagline "An AI-Native Textbook for the Future of Robotics"

## Alternatives Considered

- **Keep narrow ROS 2 branding** → rejected (misrepresents scope, doesn't align with full curriculum)
- **Multiple top-level modules without parent** → rejected (less organized, doesn't provide clear textbook structure)
- **No branding update** → rejected (poor submission quality, misaligned with hackathon requirements)

## Consequences

**Positive:**
- Accurate, professional representation of full textbook
- Stronger hackathon submission impact
- Better user orientation
- Clear curriculum structure that matches the complete scope of the textbook
- Improved alignment with hackathon theme and judging criteria

**Negative:**
- One-time config and content update required
- Minor disruption during transition period

## References

- [frontend-book/docusaurus.config.ts](../frontend-book/docusaurus.config.ts)
- [frontend-book/src/pages/index.tsx](../frontend-book/src/pages/index.tsx)
- [frontend-book/sidebars.ts](../frontend-book/sidebars.ts)