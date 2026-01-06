"""
Web crawling functionality for the URL Ingestion & Embedding Pipeline
"""
import requests
from bs4 import BeautifulSoup
from typing import List, Optional, Tuple, Dict, Any
import time
import logging
from urllib.parse import urljoin, urlparse
from dataclasses import dataclass
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

from .utils import generate_uuid, validate_url
from .models import DocumentChunk


@dataclass
class CrawlResult:
    """Represents the result of a crawl operation."""
    url: str
    success: bool
    content: Optional[str] = None
    status_code: Optional[int] = None
    error_message: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None


@dataclass
class CrawlConfiguration:
    """Configuration for crawling behavior."""
    delay_between_requests: float = 1.0  # seconds
    timeout: int = 30
    max_retries: int = 3
    respect_robots_txt: bool = False  # Not implemented in this basic version
    user_agent: str = "Mozilla/5.0 (compatible; DocumentCrawler/1.0)"
    max_content_length: int = 1024 * 1024  # 1MB limit


def create_session_with_retries(
    retries: int = 3,
    backoff_factor: float = 0.3,
    status_forcelist: Tuple[int, ...] = (500, 502, 504)
) -> requests.Session:
    """
    Create a requests session with retry strategy.

    Args:
        retries: Number of retries
        backoff_factor: Backoff factor for retries
        status_forcelist: HTTP status codes to retry on

    Returns:
        Configured requests.Session object
    """
    session = requests.Session()
    retry = Retry(
        total=retries,
        read=retries,
        connect=retries,
        backoff_factor=backoff_factor,
        status_forcelist=status_forcelist,
        allowed_methods=["HEAD", "GET", "OPTIONS"]
    )
    adapter = HTTPAdapter(max_retries=retry)
    session.mount("http://", adapter)
    session.mount("https://", adapter)
    return session


def crawl_url(
    url: str,
    config: CrawlConfiguration = None
) -> CrawlResult:
    """
    Fetch content from a single URL with error handling.

    Args:
        url: URL to crawl
        config: Crawl configuration (uses defaults if None)

    Returns:
        CrawlResult object with crawl results
    """
    if config is None:
        config = CrawlConfiguration()

    if not validate_url(url):
        return CrawlResult(
            url=url,
            success=False,
            error_message=f"Invalid URL format: {url}"
        )

    session = create_session_with_retries(
        retries=config.max_retries
    )

    headers = {
        'User-Agent': config.user_agent,
        'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8',
        'Accept-Language': 'en-US,en;q=0.5',
        'Accept-Encoding': 'gzip, deflate',
        'Connection': 'keep-alive',
    }

    try:
        response = session.get(
            url,
            headers=headers,
            timeout=config.timeout
        )

        if response.status_code == 200:
            # Check content length to avoid extremely large responses
            content_length = len(response.content)
            if content_length > config.max_content_length:
                return CrawlResult(
                    url=url,
                    success=False,
                    status_code=response.status_code,
                    error_message=f"Content too large: {content_length} bytes"
                )

            # Try to decode content properly
            try:
                content = response.text
            except UnicodeDecodeError:
                # Fallback to encoding detection
                content = response.content.decode(response.apparent_encoding)

            return CrawlResult(
                url=url,
                success=True,
                content=content,
                status_code=response.status_code,
                metadata={
                    'content_type': response.headers.get('content-type', ''),
                    'content_length': content_length,
                    'encoding': response.apparent_encoding
                }
            )
        else:
            return CrawlResult(
                url=url,
                success=False,
                status_code=response.status_code,
                error_message=f"HTTP {response.status_code}: {response.reason}"
            )

    except requests.exceptions.Timeout:
        return CrawlResult(
            url=url,
            success=False,
            error_message=f"Request timed out after {config.timeout} seconds"
        )
    except requests.exceptions.ConnectionError as e:
        return CrawlResult(
            url=url,
            success=False,
            error_message=f"Connection error: {str(e)}"
        )
    except requests.exceptions.RequestException as e:
        return CrawlResult(
            url=url,
            success=False,
            error_message=f"Request error: {str(e)}"
        )
    except Exception as e:
        return CrawlResult(
            url=url,
            success=False,
            error_message=f"Unexpected error: {str(e)}"
        )
    finally:
        session.close()


def extract_content_from_html(
    html_content: str,
    url: str
) -> Tuple[str, Dict[str, Any]]:
    """
    Extract clean text content and metadata from HTML.

    Args:
        html_content: Raw HTML content
        url: Source URL for the content

    Returns:
        Tuple of (clean_text, metadata_dict)
    """
    soup = BeautifulSoup(html_content, 'html.parser')

    # Remove script and style elements
    for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
        script.decompose()

    # Extract metadata
    metadata = {
        'url': url,
        'title': '',
        'description': '',
        'author': '',
        'keywords': [],
        'language': 'unknown'
    }

    # Extract title
    title_tag = soup.find('title')
    if title_tag:
        metadata['title'] = title_tag.get_text().strip()

    # Extract meta description
    desc_tag = soup.find('meta', attrs={'name': 'description'})
    if desc_tag and desc_tag.get('content'):
        metadata['description'] = desc_tag.get('content').strip()

    # Extract meta author
    author_tag = soup.find('meta', attrs={'name': 'author'})
    if author_tag and author_tag.get('content'):
        metadata['author'] = author_tag.get('content').strip()

    # Extract meta keywords
    keywords_tag = soup.find('meta', attrs={'name': 'keywords'})
    if keywords_tag and keywords_tag.get('content'):
        metadata['keywords'] = [
            kw.strip() for kw in keywords_tag.get('content').split(',') if kw.strip()
        ]

    # Extract language
    html_tag = soup.find('html')
    if html_tag and html_tag.get('lang'):
        metadata['language'] = html_tag.get('lang')

    # Try to find main content area (common selectors for documentation sites)
    main_content = (
        soup.find('main') or
        soup.find('article') or
        soup.find('div', class_='main-wrapper') or
        soup.find('div', {'role': 'main'}) or
        soup.find('div', class_='container') or
        soup.find('div', class_='theme-doc-markdown') or
        soup.find('div', class_='doc-content') or
        soup.find('div', class_='content') or
        soup.body
    )

    if main_content:
        # Extract text content
        text_content = main_content.get_text(separator=' ', strip=True)
    else:
        # Fallback to body content
        body = soup.find('body')
        text_content = body.get_text(separator=' ', strip=True) if body else ""

    # Clean up excessive whitespace
    import re
    text_content = re.sub(r'\s+', ' ', text_content).strip()

    return text_content, metadata


def crawl_and_process_url(
    url: str,
    config: CrawlConfiguration = None
) -> Optional[DocumentChunk]:
    """
    Crawl a URL and return a processed DocumentChunk.

    Args:
        url: URL to crawl and process
        config: Crawl configuration

    Returns:
        DocumentChunk object or None if crawling failed
    """
    if config is None:
        config = CrawlConfiguration()

    # Crawl the URL
    crawl_result = crawl_url(url, config)

    if not crawl_result.success or not crawl_result.content:
        logging.error(f"Failed to crawl {url}: {crawl_result.error_message}")
        return None

    # Extract content and metadata
    clean_content, metadata = extract_content_from_html(crawl_result.content, url)

    if not clean_content:
        logging.warning(f"No content extracted from {url}")
        return None

    # Create DocumentChunk
    chunk = DocumentChunk(
        chunk_id=generate_uuid(),
        content=clean_content,
        source_url=url,
        title=metadata.get('title', ''),
        metadata=metadata
    )

    return chunk


def crawl_multiple_urls(
    urls: List[str],
    config: CrawlConfiguration = None
) -> List[DocumentChunk]:
    """
    Crawl multiple URLs with appropriate delays between requests.

    Args:
        urls: List of URLs to crawl
        config: Crawl configuration

    Returns:
        List of DocumentChunk objects
    """
    if config is None:
        config = CrawlConfiguration()

    chunks = []
    session = create_session_with_retries(retries=config.max_retries)

    for i, url in enumerate(urls):
        logging.info(f"Crawling URL {i+1}/{len(urls)}: {url}")

        # Crawl the URL
        chunk = crawl_and_process_url(url, config)

        if chunk:
            chunks.append(chunk)
            logging.info(f"Successfully crawled {url}, extracted {len(chunk.content)} characters")
        else:
            logging.error(f"Failed to crawl {url}")

        # Add delay between requests to be respectful to the server
        if i < len(urls) - 1:  # Don't delay after the last request
            time.sleep(config.delay_between_requests)

    session.close()
    return chunks


def detect_language(text: str) -> str:
    """
    Detect the language of the text content.

    Args:
        text: Text content to analyze

    Returns:
        Detected language code (e.g., 'en', 'es', 'fr', etc.)
    """
    try:
        # This is a simple heuristic-based approach
        # For production use, consider using langdetect or similar library
        import re

        # Count characters from different scripts
        latin_chars = len(re.findall(r'[a-zA-Z]', text))
        cyrillic_chars = len(re.findall(r'[а-яА-Я]', text))
        chinese_chars = len(re.findall(r'[\u4e00-\u9fff]', text))
        arabic_chars = len(re.findall(r'[\u0600-\u06ff]', text))
        japanese_chars = len(re.findall(r'[\u3040-\u309f\u30a0-\u30ff]', text))  # Hiragana and Katakana

        total_chars = latin_chars + cyrillic_chars + chinese_chars + arabic_chars + japanese_chars

        if total_chars == 0:
            return 'unknown'

        # Simple language detection based on character scripts
        if chinese_chars / total_chars > 0.3:
            return 'zh'
        elif cyrillic_chars / total_chars > 0.3:
            return 'ru'
        elif arabic_chars / total_chars > 0.3:
            return 'ar'
        elif japanese_chars / total_chars > 0.3:
            return 'ja'
        elif latin_chars / total_chars > 0.7:  # If mostly Latin characters
            return 'en'
        else:
            return 'unknown'
    except:
        # If detection fails, return unknown
        return 'unknown'


# Example usage function
def main():
    """Example usage of the crawler functionality."""
    urls = [
        "https://example.com/page1",
        "https://example.com/page2"
    ]

    config = CrawlConfiguration(
        delay_between_requests=1.0,
        timeout=30,
        max_retries=3
    )

    print("Starting crawl of example URLs...")
    chunks = crawl_multiple_urls(urls, config)

    for chunk in chunks:
        print(f"URL: {chunk.source_url}")
        print(f"Title: {chunk.title}")
        print(f"Content length: {len(chunk.content)} characters")
        print("---")


if __name__ == "__main__":
    main()