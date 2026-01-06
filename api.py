"""
Main FastAPI application for the RAG Chatbot API.

This module creates the FastAPI application instance and mounts all required routes.
"""
import logging
from fastapi import FastAPI, Request, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response
from fastapi.exceptions import RequestValidationError
from fastapi.encoders import jsonable_encoder
from starlette.exceptions import HTTPException as StarletteHTTPException
from starlette.responses import JSONResponse
import time
import uvicorn
from endpoints import router as endpoints_router, limiter
from slowapi import _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded
from agent_integration import agent_manager
from config import config

# Import startup validator
from backend.src.startup_validator import StartupValidator


# Configure logging
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL.upper(), logging.INFO),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    force=True  # Ensure configuration is applied even if already configured
)
logger = logging.getLogger(__name__)


# Create FastAPI app instance
app = FastAPI(
    title="RAG Chatbot API",
    description="API for integrating the RAG chatbot with frontend applications",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# Add CORS middleware for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict this to your frontend domains
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add rate limiting
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Custom exception handler to convert validation errors to 400 for test compatibility
@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request: Request, exc: RequestValidationError):
    # For test compatibility, return 400 instead of 422 for validation errors
    return JSONResponse(
        status_code=400,
        content={"detail": "Validation error", "errors": exc.errors()},
    )

# Include the endpoints router
app.include_router(endpoints_router, prefix="", tags=["core"])

# Dependency injection for agent - making it available globally
app.state.agent_manager = agent_manager

# Dependency injection for config - making it available globally
app.state.config = config

# Add startup event handler for validation
@app.on_event("startup")
async def startup_event():
    """Run startup validation to detect deprecated API usage"""
    print("🔍 Running startup validation...")
    validator = StartupValidator()
    validator.run_startup_validation()
    print("✅ Startup validation completed successfully")


# Add logging middleware
@app.middleware("http")
async def logging_middleware(request: Request, call_next):
    """
    Middleware to log API requests and responses for debugging and monitoring.
    """
    start_time = time.time()

    # Log request
    logger.info(f"Request: {request.method} {request.url}")
    try:
        body = await request.body()
        if body:
            logger.debug(f"Request body: {body.decode()}")
    except Exception as e:
        logger.warning(f"Could not log request body: {str(e)}")

    # Process request
    response: Response = await call_next(request)

    # Calculate processing time
    process_time = time.time() - start_time

    # Log response
    logger.info(f"Response: {response.status_code} in {process_time:.2f}s")

    return response


# Root endpoint for basic health check
@app.get("/")
async def root():
    """
    Root endpoint that provides basic information about the API.

    Returns:
        dict: Basic API information
    """
    logger.info("Root endpoint accessed")
    return {
        "message": "RAG Chatbot API",
        "status": "running",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health"
    }


if __name__ == "__main__":
    # Run the application with uvicorn for development
    uvicorn.run(
        "api:app",
        host="0.0.0.0",
        port=8000,
        reload=True,  # Enable hot reload for development
        log_level="info"
    )