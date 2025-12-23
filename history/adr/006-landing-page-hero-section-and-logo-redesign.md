# ADR-006: Landing Page Hero Section and Logo Redesign

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-23
- **Feature:** 001-hero-section-redesign
- **Context:** Current landing page uses default Docusaurus template with minimal styling and generic swan logo. The Physical AI & Humanoid Robotics textbook needs a professional, futuristic first impression that immediately conveys the advanced nature of the content to engage AI students and developers. The hero section is critical for hackathon judging and user engagement.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement a comprehensive hero section redesign with:

- **Layout**: Full-width hero section with dark gradient background (#0f172a or #111)
- **Typography**: Large centered heading "Physical AI & Humanoid Robotics", subtitle "The Complete AI-Native Textbook", tagline "Master ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action Systems"
- **Logo**: Custom futuristic logo featuring humanoid robot silhouette with glowing cyan neural circuits, encircled by metallic silver ring, PNG format with transparent background
- **Call-to-Action**: "Start Reading →" button linking to first chapter
- **Styling**: Tailwind CSS for styling with cyan (#00d4ff) accent colors and white/silver text
- **Animation**: Subtle CSS-based glow effects (no external dependencies)
- **Configuration**: Update docusaurus.config.ts to use new logo in navbar and favicon

## Consequences

### Positive

- Strong visual identity and professional impression for hackathon judging
- Higher user retention and engagement with compelling first impression
- Aligns perfectly with "AI-Native Textbook" vision and futuristic theme
- Improved brand recognition with custom logo distinct from default Docusaurus
- Consistent theming across landing page and navigation elements
- Fast loading with CSS animations instead of external dependencies

### Negative

- Additional design and implementation effort required
- Custom logo asset needs to be created and maintained
- Potential for increased complexity in responsive layouts
- Departure from standard Docusaurus styling may require more maintenance
- Risk of performance impact if animations are not optimized

## Alternatives Considered

**Keep Default Docusaurus Template**: Continue using generic Docusaurus swan logo and basic template → rejected (no branding identity, low impact for hackathon, doesn't represent Physical AI scope)

**Generic Hero Section**: Implement basic hero with simple background and text → rejected (low visual impact, doesn't convey futuristic/advanced nature of content)

**Heavy Animation Approach**: Implement complex JavaScript animations and effects → rejected (performance risk, complexity, potential accessibility issues)

## References

- Feature Spec: [Link to spec.md](../specs/001-hero-section-redesign/spec.md)
- Implementation Plan: [Link to plan.md](../specs/001-hero-section-redesign/plan.md)
- Related ADRs: ADR-005-full-rebranding-physical-ai-humanoid-robotics-textbook.md (complementary rebranding effort)
- Evaluator Evidence: [Link to eval notes/PHR showing graders and outcomes](../specs/001-hero-section-redesign/research.md)