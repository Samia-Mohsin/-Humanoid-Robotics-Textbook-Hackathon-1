# ADR-004: Sidebar Restructuring for Module-Based Navigation Hierarchy

**Status**: Accepted
**Date**: 2025-12-21

## Context

- Current sidebar shows flat or cluttered list of individual chapters
- Textbook curriculum is organized into 4 clear modules
- Users expect to see high-level modules first, then drill down into chapters
- Project: Physical AI & Humanoid Robotics Textbook Hackathon
- Docusaurus classic template with auto-generated or manual sidebar showing individual chapter names directly under docs (e.g., "ROS 2 Basics", "Communication Model", etc.), making sidebar cluttered

## Decision

Replace current sidebar configuration with manual, hierarchical structure:

- Use sidebars.ts with category-based grouping
- 4 top-level collapsible categories matching module titles:
  1. "Module 1: The Robotic Nervous System (ROS 2)"
  2. "Module 2: Advanced Digital Twin Integration (Gazebo & Unity)"
  3. "Module 3: The AI-Robot Brain (NVIDIA Isaac™)"
  4. "Module 4: Vision-Language-Action (VLA)"
- All existing docs files nested correctly under respective modules
- Disable autogenerate if currently enabled
- Optional: Add module links to navbar for quick access

## Alternatives Considered

- **Keep flat structure** → rejected (poor UX, cluttered, doesn't align with curriculum)
- **Auto-generation with categories** → rejected (less control, harder to customize, may not align with curriculum structure)
- **Remove sidebar** → rejected (bad for documentation navigation, essential for textbook structure)

## Consequences

**Positive:**
- Clean, professional, curriculum-aligned navigation
- Better user orientation and progress perception
- Easier to add module-specific features later (e.g., progress circles, completion tracking)
- Clear learning path aligned with textbook modules
- Reduced visual noise in sidebar

**Negative:**
- Requires manual sidebar maintenance (acceptable for static textbook)
- Slight increase in navigation depth (one extra click to reach chapters)

## References

- [frontend-book/sidebars.ts](../frontend-book/sidebars.ts)
- [frontend-book/docusaurus.config.ts](../frontend-book/docusaurus.config.ts)
- [frontend-book/docs/](../frontend-book/docs/)