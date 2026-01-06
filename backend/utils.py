"""
Utility functions for the URL Ingestion & Embedding Pipeline
"""
import uuid
import logging
from typing import Any, Optional


def setup_logging(level: str = "INFO") -> logging.Logger:
    """Set up logging configuration."""
    logger = logging.getLogger(__name__)
    logger.setLevel(getattr(logging, level.upper()))

    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger


def generate_uuid() -> str:
    """Generate a UUID string for chunk and embedding IDs."""
    return str(uuid.uuid4())


def safe_execute(func, *args, **kwargs) -> tuple[bool, Optional[Any], Optional[Exception]]:
    """Safely execute a function and return success status, result, and exception."""
    try:
        result = func(*args, **kwargs)
        return True, result, None
    except Exception as e:
        return False, None, e


def validate_url(url: str) -> bool:
    """Basic URL validation."""
    import re
    pattern = re.compile(
        r'^https?://'  # http:// or https://
        r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
        r'localhost|'  # localhost...
        r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
        r'(?::\d+)?'  # optional port
        r'(?:/?|[/?]\S+)$', re.IGNORECASE)
    return url is not None and pattern.search(url) is not None


def generate_deterministic_id(source_url: str, chunk_index: int) -> str:
    """
    Generate a deterministic ID based on source URL and chunk index.
    Creates a UUID from the combination of source URL and chunk index.

    Args:
        source_url: The original URL where the content was extracted from
        chunk_index: The position of this chunk within the original document

    Returns:
        Deterministic UUID string based on source URL and chunk index
    """
    import hashlib
    import uuid
    # Create a deterministic string from the source URL and chunk index
    combined_str = f"{source_url}_{chunk_index}"
    # Create a hash of the combined string
    hash_object = hashlib.md5(combined_str.encode())
    hex_dig = hash_object.hexdigest()
    # Create a UUID from the hash ensuring it's in proper UUID format
    return str(uuid.UUID(hex=hex_dig[:32]))  # Take first 32 chars for UUID format