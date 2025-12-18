<!--
Sync Impact Report:
- Version change: N/A → 1.0.0 (initial constitution)
- Modified principles: N/A (new file)
- Added sections: All sections (new constitution)
- Removed sections: N/A
- Templates requiring updates: N/A
- Follow-up TODOs: None
-->
# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-first workflow using Spec-Kit Plus
Spec-Driven Development is mandatory: All features start with a detailed specification using Spec-Kit Plus templates; Specifications must be complete and approved before implementation begins; Clear requirements, acceptance criteria, and test scenarios required before any code is written.

### II. Technical accuracy from official sources
All content and implementations must be grounded in official documentation and authoritative sources; No hallucinated responses or fabricated information allowed; Citations and references to original sources required for all technical claims and code examples.

### III. Clear, developer-focused writing
Documentation and code must be written for developers with clear explanations, runnable examples, and practical applications; Accessibility and usability are paramount; Content should solve real-world problems with step-by-step guidance.

### IV. Reproducible setup and deployment
All environments must be reproducible with documented setup procedures; Containerized deployments preferred; Infrastructure as code required; One-click deployment to GitHub Pages with CI/CD pipelines.

### V. RAG Chatbot Grounded in Book Content
The RAG chatbot must only reference content from the book or user-selected text; No external hallucinations or fabrications allowed; Responses must be traceable to specific sections of the book; Accuracy and faithfulness to source material are non-negotiable.

### VI. GitHub-based source control and collaboration
All code and content must be managed through GitHub with proper branching strategies; Pull requests required for all changes; Code reviews mandatory for merging; Maintaining clean commit history and descriptive messages.

## Technology Stack Standards
Docusaurus for documentation, OpenAI Agents/ChatKit for RAG functionality, FastAPI for backend services, Neon Postgres for vector storage, and Qdrant Cloud for similarity search; All technologies must be well-documented, actively maintained, and suitable for production deployment; Consistent tooling across the team with shared configurations.

## Development Workflow
Specifications created using Spec-Kit Plus templates with detailed requirements and acceptance criteria; Test-driven development approach where applicable; Regular reviews and approvals before implementation; Comprehensive documentation and runnable, well-documented code required for all features.

## Governance
This constitution supersedes all other development practices and standards; All pull requests must comply with these principles; Amendments require formal documentation, team approval, and migration plan if needed; All team members are responsible for upholding these standards.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
