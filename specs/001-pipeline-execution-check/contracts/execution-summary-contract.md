# Contract: Pipeline Execution Summary Format

## Overview
This contract defines the format and structure of the enhanced pipeline execution summary that will be displayed to users after pipeline execution.

## Execution Summary Format

When a pipeline execution completes, the system MUST output a summary in the following format:

```
Pipeline execution completed successfully!
Results: {
  "urls_processed": <integer>,
  "documents_extracted": <integer>,
  "chunks_created": <integer>,
  "embeddings_generated": <integer>,
  "storage_success": <boolean>,
  "execution_time_seconds": <float>,
  "status": <string>,
  "phase_metrics": {
    "fetching_time": <float>,
    "chunking_time": <float>,
    "embedding_time": <float>,
    "storage_time": <float>
  }
}
```

## Field Definitions

- `urls_processed`: Number of URLs that were submitted to the pipeline
- `documents_extracted`: Number of documents successfully extracted from URLs
- `chunks_created`: Number of text chunks created from documents
- `embeddings_generated`: Number of embeddings successfully generated
- `storage_success`: Whether all embeddings were successfully stored in the vector database
- `execution_time_seconds`: Total time taken for the entire pipeline execution in seconds
- `status`: Execution status ("completed", "failed", or "partial")
- `phase_metrics`: Object containing timing for each pipeline phase

## Validation Requirements

1. All count fields (urls_processed, documents_extracted, chunks_created, embeddings_generated) MUST be non-negative integers
2. execution_time_seconds MUST be a positive float value
3. status MUST be one of: "completed", "failed", "partial"
4. phase_metrics MUST contain positive float values for all phases when available
5. storage_success MUST accurately reflect whether all embeddings were stored successfully

## Error Format

When pipeline execution fails, the system MUST output an error message in the following format:

```
Pipeline execution failed: <error_message>
```

Where error_message is a clear, actionable description of what went wrong.