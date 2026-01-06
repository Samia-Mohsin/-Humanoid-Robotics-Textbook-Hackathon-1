import logging
from typing import Dict, Any, Optional
from fastapi import HTTPException
import traceback
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

class APIErrorHandler:
    """
    Error handling and logging infrastructure for API calls
    """

    @staticmethod
    def log_error(error: Exception, context: str = "", details: Optional[Dict[str, Any]] = None):
        """
        Log error with context and details without exposing sensitive information

        Args:
            error: The exception that occurred
            context: Context of where the error occurred
            details: Additional non-sensitive details about the error
        """
        error_details = {
            "error_type": type(error).__name__,
            "error_message": str(error),
            "context": context
        }

        if details:
            error_details.update(details)

        logger.error(f"API Error in {context}: {str(error)}")
        logger.debug(f"Full error details: {error_details}")

    @staticmethod
    def handle_openai_error(error: Exception, user_message: str = "An error occurred processing your request"):
        """
        Handle OpenAI-specific errors appropriately

        Args:
            error: The OpenAI error that occurred
            user_message: User-friendly message to return to the client

        Returns:
            HTTPException with appropriate status code and user-friendly message
        """
        error_type = type(error).__name__

        # Log the actual error with details (not exposed to user)
        APIErrorHandler.log_error(error, "OpenAI API call", {
            "error_type": error_type,
            "user_message": user_message
        })

        # Map specific errors to appropriate HTTP status codes
        if "AuthenticationError" in error_type or "PermissionDenied" in error_type:
            raise HTTPException(status_code=401, detail="Authentication failed. Please check your API key.")
        elif "RateLimitError" in error_type:
            raise HTTPException(status_code=429, detail="Rate limit exceeded. Please try again later.")
        elif "InvalidRequestError" in error_type:
            raise HTTPException(status_code=400, detail="Invalid request. Please check your input.")
        else:
            # For all other errors, return a generic message to avoid exposing internal details
            raise HTTPException(status_code=500, detail=user_message)

    @staticmethod
    def handle_retrieval_error(error: Exception):
        """
        Handle errors during content retrieval

        Args:
            error: The error that occurred during retrieval
        """
        APIErrorHandler.log_error(error, "Content retrieval")
        # For retrieval errors, we can still attempt to respond to the user
        # rather than failing completely
        logger.warning(f"Content retrieval failed: {str(error)}. Proceeding with general response.")
        return []

    @staticmethod
    def create_safe_error_response(error: Exception, context: str = ""):
        """
        Create a safe error response that doesn't expose internal details

        Args:
            error: The error that occurred
            context: Context where error occurred

        Returns:
            Dictionary with safe error information
        """
        APIErrorHandler.log_error(error, context)

        return {
            "error": "An error occurred while processing your request",
            "context": context,
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }