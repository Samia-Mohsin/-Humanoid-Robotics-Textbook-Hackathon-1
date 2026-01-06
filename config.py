"""
Configuration module for the FastAPI RAG Integration.

This module handles loading configuration settings and environment variables
needed for the application to run properly.
"""
import os
from typing import Optional
from dotenv import load_dotenv


# Load environment variables from .env file
load_dotenv()


class Config:
    """
    Configuration class that holds all application settings.
    """

    # Qdrant configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_docs")

    # Cohere configuration
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")

    # OpenAI configuration
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")

    # Application configuration
    DEBUG_MODE: bool = os.getenv("DEBUG", "False").lower() == "true"
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")
    HOST: str = os.getenv("HOST", "0.0.0.0")
    PORT: int = int(os.getenv("PORT", "8000"))

    # Agent configuration
    AGENT_MODEL: str = os.getenv("AGENT_MODEL", "gpt-3.5-turbo")

    # Rate limiting and performance
    MAX_CONCURRENT_REQUESTS: int = int(os.getenv("MAX_CONCURRENT_REQUESTS", "10"))
    REQUEST_TIMEOUT: int = int(os.getenv("REQUEST_TIMEOUT", "30"))

    @classmethod
    def validate(cls) -> tuple[bool, list[str]]:
        """
        Validate that all required configuration values are present.

        Returns:
            Tuple of (is_valid, list_of_missing_variables)
        """
        missing_vars = []

        # Check required environment variables
        if not cls.QDRANT_URL:
            missing_vars.append("QDRANT_URL")

        if not cls.QDRANT_API_KEY:
            missing_vars.append("QDRANT_API_KEY")

        if not cls.QDRANT_COLLECTION_NAME:
            missing_vars.append("QDRANT_COLLECTION_NAME")

        if not cls.COHERE_API_KEY:
            missing_vars.append("COHERE_API_KEY")

        if not cls.OPENAI_API_KEY:
            missing_vars.append("OPENAI_API_KEY")

        return len(missing_vars) == 0, missing_vars

    @classmethod
    def get_db_config(cls) -> dict:
        """
        Get database (Qdrant) configuration as a dictionary.

        Returns:
            Dictionary containing Qdrant configuration
        """
        return {
            "url": cls.QDRANT_URL,
            "api_key": cls.QDRANT_API_KEY,
            "collection_name": cls.QDRANT_COLLECTION_NAME
        }

    @classmethod
    def get_agent_config(cls) -> dict:
        """
        Get agent configuration as a dictionary.

        Returns:
            Dictionary containing agent configuration
        """
        return {
            "model": cls.AGENT_MODEL,
            "openai_api_key": cls.OPENAI_API_KEY,
            "cohere_api_key": cls.COHERE_API_KEY
        }


# Global config instance
config = Config()


def get_config() -> Config:
    """
    Get the global configuration instance.

    Returns:
        Config: The global configuration instance
    """
    return config