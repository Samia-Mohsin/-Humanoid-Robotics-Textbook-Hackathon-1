# Data Model: Sitemap-Based Content Ingestion Pipeline

## Entity: SitemapURL
**Description**: A URL extracted from the sitemap.xml file that needs to be processed

**Fields**:
- `url`: String - The URL extracted from the sitemap
- `last_modified`: DateTime - When the URL was last modified (from sitemap if available)
- `change_frequency`: String - How frequently the URL is expected to change (from sitemap if available)
- `priority`: Float - Priority of the URL relative to other URLs (from sitemap if available)
- `status`: String - Processing status ("pending", "processing", "completed", "failed")
- `error_message`: String - Error message if processing failed

**Validation Rules**:
- url must be a valid URL format
- priority must be between 0.0 and 1.0 if specified
- status must be one of the allowed values

## Entity: SitemapIngestionResult
**Description**: The outcome of processing a sitemap, including success/failure counts and error details

**Fields**:
- `total_urls`: Integer - Total number of URLs extracted from the sitemap
- `successful_processing`: Integer - Number of URLs successfully processed
- `failed_processing`: Integer - Number of URLs that failed processing
- `processed_urls`: Array - List of SitemapURL objects with their status
- `start_time`: DateTime - When the sitemap processing started
- `end_time`: DateTime - When the sitemap processing completed
- `total_duration`: Float - Total duration of processing in seconds
- `error_details`: Array - Details about any errors that occurred during processing

**Validation Rules**:
- total_urls must equal successful_processing + failed_processing
- start_time must be before end_time
- All count fields must be non-negative integers

## Entity: BulkProcessingJob
**Description**: A job that manages the processing of multiple URLs extracted from the sitemap

**Fields**:
- `job_id`: String - Unique identifier for the processing job
- `sitemap_url`: String - URL of the sitemap being processed
- `status`: String - Current status of the job ("pending", "in_progress", "completed", "failed")
- `progress_percentage`: Float - Completion percentage (0.0 to 100.0)
- `current_url_index`: Integer - Index of the current URL being processed
- `total_urls`: Integer - Total number of URLs to process
- `processed_count`: Integer - Number of URLs processed so far
- `successful_count`: Integer - Number of successfully processed URLs so far
- `failed_count`: Integer - Number of failed URLs so far
- `start_time`: DateTime - When the job started
- `last_update_time`: DateTime - When the job status was last updated
- `rate_limit_delay`: Float - Delay between requests in seconds to respect rate limits

**Validation Rules**:
- progress_percentage must be between 0.0 and 100.0
- current_url_index must be less than or equal to total_urls
- processed_count must equal successful_count + failed_count
- status must be one of the allowed values
- rate_limit_delay must be positive

## Relationships
- Each SitemapIngestionResult contains multiple SitemapURL objects in the processed_urls array
- Each BulkProcessingJob manages the processing of multiple SitemapURL objects
- SitemapIngestionResult represents the final outcome of a completed BulkProcessingJob