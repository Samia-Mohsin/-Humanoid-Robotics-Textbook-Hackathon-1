"""
Configuration management for the URL Ingestion & Embedding Pipeline
"""
import os
from dataclasses import dataclass
from typing import Optional


@dataclass
class Config:
    """Configuration class to manage API keys and settings."""

    # Required fields (no defaults)
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str

    # Optional fields (with defaults)
    cohere_model: str = "embed-english-v3.0"
    chunk_size: int = 512
    chunk_overlap: int = 50
    # Sitemap processing configuration
    rate_limit_delay: float = 1.0
    max_urls_per_sitemap: int = 1000
    enable_progress_tracking: bool = True
    max_memory_usage_mb: int = 1024

    @classmethod
    def from_env(cls) -> 'Config':
        """Create Config instance from environment variables."""
        cohere_api_key = os.getenv('COHERE_API_KEY')
        qdrant_url = os.getenv('QDRANT_URL')
        qdrant_api_key = os.getenv('QDRANT_API_KEY')

        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        if not qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")
        if not qdrant_api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")

        chunk_size = int(os.getenv('CHUNK_SIZE', '512'))
        chunk_overlap = int(os.getenv('CHUNK_OVERLAP', '50'))
        cohere_model = os.getenv('COHERE_MODEL', 'embed-english-v3.0')
        rate_limit_delay = float(os.getenv('RATE_LIMIT_DELAY', '1.0'))
        max_urls_per_sitemap = int(os.getenv('MAX_URLS_PER_SITEMAP', '1000'))
        enable_progress_tracking = os.getenv('ENABLE_PROGRESS_TRACKING', 'true').lower() == 'true'
        max_memory_usage_mb = int(os.getenv('MAX_MEMORY_USAGE_MB', '1024'))

        return cls(
            cohere_api_key=cohere_api_key,
            qdrant_url=qdrant_url,
            qdrant_api_key=qdrant_api_key,
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap,
            cohere_model=cohere_model,
            rate_limit_delay=rate_limit_delay,
            max_urls_per_sitemap=max_urls_per_sitemap,
            enable_progress_tracking=enable_progress_tracking,
            max_memory_usage_mb=max_memory_usage_mb
        )


def validate_config(config: Config) -> list[str]:
    """Validate configuration parameters and return list of errors."""
    errors = []

    # Validate chunk size
    if config.chunk_size <= 0:
        errors.append("chunk_size must be positive")

    # Validate chunk overlap
    if config.chunk_overlap >= config.chunk_size:
        errors.append("chunk_overlap must be less than chunk_size")

    # Validate URLs (basic check)
    if not config.qdrant_url.startswith(('http://', 'https://')):
        errors.append("qdrant_url must be a valid URL starting with http:// or https://")

    return errors