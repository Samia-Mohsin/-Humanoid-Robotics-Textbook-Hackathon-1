# Data Model: RAG Retrieval Validation

## Core Entities

### Query
- **Description**: Text input from user requesting relevant content from the humanoid robotics textbook
- **Fields**:
  - `text`: string - The query text to search for relevant content
  - `top_k`: integer - Number of top results to retrieve (default: 5)
  - `collection_name`: string - Name of the Qdrant collection to search in

### SearchResult
- **Description**: Contains the results of a similarity search operation
- **Fields**:
  - `id`: string - Unique identifier for the retrieved chunk
  - `text`: string - The actual text content of the retrieved chunk
  - `score`: float - Similarity score for the retrieved chunk
  - `metadata`: dict - Additional metadata including source URL and document information
  - `source_url`: string - URL of the original document where the content came from

### RetrievalResult
- **Description**: Aggregated results from a retrieval operation
- **Fields**:
  - `query`: string - The original query text
  - `results`: list of SearchResult - List of retrieved content chunks
  - `total_results`: integer - Total number of results returned
  - `execution_time`: float - Time taken to execute the search in seconds
  - `validation_passed`: boolean - Whether the validation checks passed

### ValidationReport
- **Description**: Report of validation checks performed on retrieval results
- **Fields**:
  - `source_url_match_count`: integer - Number of results with matching source URLs
  - `metadata_integrity_check`: boolean - Whether all metadata fields are present and valid
  - `content_relevance_score`: float - Average relevance score of retrieved content
  - `issues_found`: list of strings - List of any issues found during validation
  - `overall_status`: string - Overall validation status (pass/warning/fail)

## Relationships
- A `Query` generates one `RetrievalResult`
- A `RetrievalResult` contains multiple `SearchResult` items
- A `RetrievalResult` has one `ValidationReport`
- Each `SearchResult` has metadata that includes source information

## Validation Rules
- `Query.text` must not be empty
- `Query.top_k` must be between 1 and 100
- `SearchResult.score` must be between 0 and 1
- `SearchResult.metadata` must contain required fields (source_url, document_id)
- `ValidationReport.overall_status` must be one of: "pass", "warning", "fail"