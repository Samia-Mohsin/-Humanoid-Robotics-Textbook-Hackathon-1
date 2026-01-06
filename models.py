"""
Pydantic models for the FastAPI RAG Integration API

Contains request and response models for the RAG chatbot API endpoints.
"""
from pydantic import BaseModel, Field
from typing import Optional, List
from uuid import UUID


class QueryRequest(BaseModel):
    """
    Request model for the query endpoint.

    Attributes:
        query: The user's question or query text (required, minimum 3 characters)
        conversation_id: Unique identifier for conversation continuity (optional, UUID format)
        debug: Flag to enable debug mode (optional, defaults to False)
    """
    query: str = Field(..., min_length=3, max_length=5000, description="The user's question or query text")
    conversation_id: Optional[str] = Field(None, description="Unique identifier for conversation continuity")
    debug: Optional[bool] = Field(False, description="Flag to enable debug mode")


class ErrorResponse(BaseModel):
    """
    Response model for error responses.

    Attributes:
        error: Human-readable error message
        error_code: Machine-readable error code (optional)
        details: Additional error details (optional)
    """
    error: str = Field(..., description="Human-readable error message")
    error_code: Optional[str] = Field(None, description="Machine-readable error code")
    details: Optional[dict] = Field(None, description="Additional error details")


class QueryResponse(BaseModel):
    """
    Response model for the query endpoint.

    Attributes:
        response: The RAG agent's response to the query
        conversation_id: Conversation identifier for continuity (optional)
        sources: List of source citations for the response (optional)
        debug_info: Debug information when debug mode is enabled (optional)
    """
    response: str = Field(..., description="The RAG agent's response to the query")
    conversation_id: Optional[str] = Field(None, description="Conversation identifier for continuity")
    sources: Optional[List[str]] = Field(None, description="List of source citations for the response")
    debug_info: Optional[dict] = Field(None, description="Debug information when debug mode is enabled")


class HealthCheckResponse(BaseModel):
    """
    Response model for the health check endpoint.

    Attributes:
        status: Health status of the service ('healthy', 'degraded', 'unhealthy')
        timestamp: ISO 8601 timestamp of the health check
        dependencies: Status of dependent services (optional)
    """
    status: str = Field(..., description="Health status of the service")
    timestamp: str = Field(..., description="ISO 8601 timestamp of the health check")
    dependencies: Optional[dict] = Field(None, description="Status of dependent services")