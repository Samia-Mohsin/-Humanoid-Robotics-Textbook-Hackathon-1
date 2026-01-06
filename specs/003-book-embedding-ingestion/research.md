# Research Findings: URL Ingestion & Embedding Pipeline

## Decision Log

### Qdrant Cloud Configuration
- **Decision**: Use Qdrant Cloud with a dedicated collection for document embeddings
- **Rationale**: Qdrant Cloud provides managed vector database service with good Python SDK support
- **Implementation**: Will use qdrant-client library with cloud endpoint and API key from environment variables
- **Alternatives considered**: Pinecone, Weaviate, local Qdrant - Qdrant Cloud chosen for simplicity and reliability

### Cohere API Integration
- **Decision**: Use Cohere's embed-english-v3.0 model for text embeddings
- **Rationale**: Cohere provides high-quality embeddings with good documentation and Python SDK
- **Implementation**: Will use cohere library with API key from environment variables
- **Alternatives considered**: OpenAI embeddings, Hugging Face models - Cohere chosen as per requirements

### Docusaurus Content Extraction
- **Decision**: Use requests + BeautifulSoup4 for web scraping, focusing on article/main content tags
- **Rationale**: Docusaurus sites have consistent HTML structure with content in main/article tags
- **Implementation**: Extract text from main content areas, ignore navigation and sidebar elements
- **Alternatives considered**: Selenium, Playwright - requests/BeautifulSoup chosen for simplicity

### Text Chunking Parameters
- **Decision**: Use 512-token chunks with 50-token overlap for optimal context preservation
- **Rationale**: Balances semantic coherence with retrieval precision; fits well within embedding model limits
- **Implementation**: Split by sentences while maintaining chunk size constraints
- **Alternatives considered**: Fixed character counts, paragraph-based chunks - token-based chosen for semantic coherence

### uv Project Management
- **Decision**: Use uv for fast Python package management and virtual environment
- **Rationale**: uv provides faster dependency resolution than pip and integrates well with modern Python workflows
- **Implementation**: Initialize project with pyproject.toml configuration
- **Alternatives considered**: pip, poetry - uv chosen as per requirements