# Contract: Sitemap Processing Interface

## Overview
This contract defines the interface and behavior for the sitemap processing functionality that extracts URLs from sitemap.xml files and processes them through the ingestion pipeline.

## Sitemap Processing Function Interface

### Function: process_sitemap
Processes all URLs from a sitemap.xml file through the content ingestion pipeline.

**Input Parameters**:
- `sitemap_url`: String - URL of the sitemap.xml file to process
- `rate_limit_delay`: Float - Delay in seconds between requests (default: 1.0)
- `max_concurrent`: Integer - Maximum number of concurrent operations (default: 1, for rate limiting)

**Return Value**:
Returns a `SitemapIngestionResult` object with the following structure:
```
{
  "total_urls": <integer>,
  "successful_processing": <integer>,
  "failed_processing": <integer>,
  "processed_urls": [<SitemapURL>, ...],
  "start_time": <ISO datetime>,
  "end_time": <ISO datetime>,
  "total_duration": <float>,
  "error_details": [<error_object>, ...]
}
```

## Command Line Interface

### Command: --sitemap
Allows processing of sitemap URLs via command line.

**Usage**:
```
python main.py --sitemap <sitemap_url>
```

**Options**:
- `--sitemap`: URL of the sitemap.xml file to process
- `--rate-limit`: Delay in seconds between requests (default: 1.0)
- `--progress-interval`: Interval in seconds for progress updates (default: 10.0)

## Progress Reporting Interface

### Progress Updates
During sitemap processing, the system MUST output progress updates in the following format:

```
Processing sitemap: <sitemap_url>
Progress: <percentage>% (<processed_count>/<total_count>)
Time elapsed: <duration> seconds
Estimated time remaining: <estimated_remaining> seconds
```

## Error Handling Contract

### Error Format
When a URL fails to process, the system MUST log the error in the following format:

```
Failed to process URL: <url>
Error: <error_message>
Status Code: <status_code> (if applicable)
```

### Graceful Degradation
The system MUST continue processing other URLs even when individual URLs fail, and include failure details in the final result.

## Rate Limiting Contract

### Request Timing
The system MUST implement rate limiting with the following requirements:
- Minimum delay of 1 second between requests (configurable)
- No more than 1 request per second to respect source website
- Delay applied after each request, not before

## Validation Requirements

1. The sitemap_url parameter MUST be a valid URL format
2. The sitemap file MUST be a valid XML sitemap format
3. All extracted URLs MUST be validated before processing
4. Processing MUST continue even when individual URLs fail
5. Progress reporting MUST update at least every 10 seconds during processing
6. Rate limiting MUST be enforced to respect the source website
7. All successful content MUST be stored in the vector database
8. Error details MUST be collected for failed URLs without stopping the process