"""
Web scraping functionality for Docusaurus sites
"""
import requests
from bs4 import BeautifulSoup
from typing import List, Optional, Tuple
import time
from urllib.parse import urljoin, urlparse
import logging

from models import DocumentChunk
from utils import safe_execute, validate_url


def fetch_content(url: str, timeout: int = 30) -> Tuple[bool, Optional[str], Optional[str]]:
    """
    Implement URL fetching function using requests.

    Args:
        url: URL to fetch content from
        timeout: Request timeout in seconds

    Returns:
        Tuple of (success, content, error_message)
    """
    if not validate_url(url):
        return False, None, f"Invalid URL format: {url}"

    try:
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }
        response = requests.get(url, headers=headers, timeout=timeout)

        if response.status_code == 200:
            return True, response.text, None
        else:
            return False, None, f"HTTP {response.status_code}: Failed to fetch {url}"
    except requests.exceptions.RequestException as e:
        return False, None, f"Request error for {url}: {str(e)}"
    except Exception as e:
        return False, None, f"Unexpected error for {url}: {str(e)}"


def parse_html_content(html_content: str) -> Optional[BeautifulSoup]:
    """
    Create HTML parsing function with BeautifulSoup4.

    Args:
        html_content: Raw HTML content to parse

    Returns:
        BeautifulSoup object or None if parsing fails
    """
    try:
        soup = BeautifulSoup(html_content, 'html.parser')
        return soup
    except Exception as e:
        logging.error(f"HTML parsing error: {str(e)}")
        return None


def extract_content_from_soup(soup: BeautifulSoup, url: str) -> Tuple[str, str]:
    """
    Implement content extraction focusing on main/article tags.

    Args:
        soup: BeautifulSoup object with parsed HTML
        url: Source URL for the content

    Returns:
        Tuple of (extracted_text, page_title)
    """
    # Remove script and style elements
    for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
        script.decompose()

    # Try to find main content area (Docusaurus specific selectors)
    main_content = (
        soup.find('main') or
        soup.find('article') or
        soup.find('div', class_='main-wrapper') or
        soup.find('div', {'role': 'main'}) or
        soup.find('div', class_='container') or
        soup.find('div', class_='theme-doc-markdown') or
        soup.body
    )

    if main_content:
        # Extract text content
        text_content = main_content.get_text(separator=' ', strip=True)

        # Clean up excessive whitespace
        import re
        text_content = re.sub(r'\s+', ' ', text_content).strip()
    else:
        # Fallback to body content
        body = soup.find('body')
        text_content = body.get_text(separator=' ', strip=True) if body else ""
        import re
        text_content = re.sub(r'\s+', ' ', text_content).strip()

    # Extract page title
    title_tag = soup.find('title')
    page_title = title_tag.get_text().strip() if title_tag else urlparse(url).path.split('/')[-1] or "Untitled"

    return text_content, page_title


def remove_html_tags_and_navigation(content: str) -> str:
    """
    Create function to remove HTML tags and navigation elements.

    Args:
        content: Raw HTML content

    Returns:
        Clean text content without HTML tags
    """
    soup = BeautifulSoup(content, 'html.parser')

    # Remove common navigation and non-content elements
    for element in soup(["script", "style", "nav", "header", "footer", "aside", "form"]):
        element.decompose()

    # Get text content
    text = soup.get_text(separator=' ', strip=True)

    # Clean up excessive whitespace
    import re
    text = re.sub(r'\s+', ' ', text).strip()

    return text


def handle_inaccessible_urls(url: str, error_msg: str) -> dict:
    """
    Implement error handling for inaccessible URLs.

    Args:
        url: The URL that failed
        error_msg: The error message

    Returns:
        Error information dictionary
    """
    error_info = {
        'url': url,
        'error': error_msg,
        'timestamp': time.time()
    }
    logging.error(f"Failed to process {url}: {error_msg}")
    return error_info


def extract_page_metadata(soup: BeautifulSoup, url: str) -> dict:
    """
    Add support for extracting page titles and metadata.

    Args:
        soup: BeautifulSoup object with parsed HTML
        url: Source URL

    Returns:
        Dictionary with page metadata
    """
    metadata = {
        'url': url,
        'title': '',
        'description': '',
        'author': '',
        'keywords': []
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
        metadata['keywords'] = [kw.strip() for kw in keywords_tag.get('content').split(',') if kw.strip()]

    return metadata


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
            # Could implement more sophisticated English detection here
            return 'en'
        else:
            return 'unknown'
    except:
        # If detection fails, return unknown
        return 'unknown'


def process_multiple_urls(urls: List[str], delay: float = 1.0) -> List[DocumentChunk]:
    """
    Create function to process multiple URLs from a list.

    Args:
        urls: List of URLs to process
        delay: Delay between requests in seconds

    Returns:
        List of DocumentChunk objects
    """
    chunks = []

    for i, url in enumerate(urls):
        logging.info(f"Processing URL {i+1}/{len(urls)}: {url}")

        success, content, error = fetch_content(url)

        if success and content:
            soup = parse_html_content(content)
            if soup:
                # Extract content and metadata
                text_content, page_title = extract_content_from_soup(soup, url)
                metadata = extract_page_metadata(soup, url)

                # Detect language of the content
                language = detect_language(text_content[:1000])  # Analyze first 1000 chars
                metadata['language'] = language

                # Create document chunk
                if text_content:
                    from utils import generate_uuid
                    chunk = DocumentChunk(
                        chunk_id=generate_uuid(),
                        content=text_content,
                        source_url=url,
                        title=page_title,
                        metadata=metadata
                    )
                    chunks.append(chunk)
                    logging.info(f"Successfully processed {url}, extracted {len(text_content)} characters in language: {language}")
                else:
                    logging.warning(f"No content extracted from {url}")
            else:
                logging.error(f"Failed to parse HTML for {url}")
                handle_inaccessible_urls(url, "Failed to parse HTML content")
        else:
            handle_inaccessible_urls(url, error or "Unknown error")

        # Add delay between requests to be respectful to the server
        if i < len(urls) - 1:  # Don't delay after the last request
            time.sleep(delay)

    return chunks