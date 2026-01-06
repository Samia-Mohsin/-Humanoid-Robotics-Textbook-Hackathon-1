# Data Model: Pipeline Execution Summary and Result Analysis

## Entity: PipelineExecutionSummary
**Description**: A summary of the pipeline execution including counts, timing, and status information

**Fields**:
- `urls_processed`: Integer - Number of URLs that were processed during execution
- `documents_extracted`: Integer - Number of documents successfully extracted from URLs
- `chunks_created`: Integer - Number of text chunks created from documents
- `embeddings_generated`: Integer - Number of embeddings generated from chunks
- `storage_success`: Boolean - Whether embeddings were successfully stored in vector database
- `execution_time_seconds`: Float - Total time taken for the entire pipeline execution
- `status`: String - Execution status ("completed", "failed", "partial")
- `timestamp`: DateTime - When the execution was completed
- `details`: Object - Additional detailed metrics for each phase

**Validation Rules**:
- All count fields must be non-negative integers
- execution_time_seconds must be positive
- status must be one of "completed", "failed", or "partial"

## Entity: ExecutionResult
**Description**: The outcome of a pipeline execution, including success/failure status and detailed metrics

**Fields**:
- `summary`: PipelineExecutionSummary - The summary information for this execution
- `error_details`: Array - Details about any errors that occurred during execution
- `partial_success_info`: Object - Information about which components succeeded when execution was partial
- `phase_metrics`: Object - Execution time and success metrics for each pipeline phase (fetching, chunking, embedding, storage)

**Validation Rules**:
- summary must be a valid PipelineExecutionSummary object
- error_details must be an array of error objects when status is "failed" or "partial"
- phase_metrics must contain metrics for all pipeline phases

## Entity: PerformanceMetrics
**Description**: Timing and efficiency measurements for different phases of the pipeline execution

**Fields**:
- `fetching_time`: Float - Time taken for URL fetching phase
- `chunking_time`: Float - Time taken for text chunking phase
- `embedding_time`: Float - Time taken for embedding generation phase
- `storage_time`: Float - Time taken for storage phase
- `total_time`: Float - Total execution time
- `throughput`: Float - Processing rate (e.g., documents per second)

**Validation Rules**:
- All time fields must be non-negative
- total_time should be approximately equal to sum of individual phases
- throughput must be positive when calculated

## Relationships
- Each ExecutionResult contains one PipelineExecutionSummary
- Each ExecutionResult includes PerformanceMetrics
- ExecutionResult may contain multiple error details in error_details array