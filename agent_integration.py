"""
Agent integration layer for the FastAPI RAG Integration.

This module provides a wrapper class for the HumanoidRoboticsAgent that handles
communication between the FastAPI endpoints and the RAG agent.
"""
from typing import Optional, Dict, Any
try:
    # Try the patched agent with local embedding fallback first
    from patched_agent import PatchedHumanoidRoboticsAgent as HumanoidRoboticsAgent
    print("Using patched agent with local embedding fallback")
except ImportError:
    # Fall back to original agent if patched version is not available
    from agent import HumanoidRoboticsAgent
    print("Using original agent")
import logging


class AgentManager:
    """
    Wrapper class for managing the HumanoidRoboticsAgent instance.

    This class handles agent initialization, cleanup, and provides a consistent
    interface for calling the agent from the API endpoints.
    """

    def __init__(self):
        """
        Initialize the AgentManager and create the agent instance.
        """
        self._agent: Optional[HumanoidRoboticsAgent] = None
        self._initialized = False
        self.logger = logging.getLogger(__name__)

    def initialize_agent(self) -> HumanoidRoboticsAgent:
        """
        Initialize the HumanoidRoboticsAgent if not already initialized.

        Returns:
            HumanoidRoboticsAgent: The initialized agent instance
        """
        if not self._initialized or self._agent is None:
            try:
                self._agent = HumanoidRoboticsAgent()
                self._initialized = True
                self.logger.info("HumanoidRoboticsAgent initialized successfully")
            except Exception as e:
                self.logger.error(f"Failed to initialize HumanoidRoboticsAgent: {str(e)}")
                raise

        return self._agent

    def safe_query_agent(self, query: str, conversation_id: Optional[str] = None, debug: bool = False) -> Dict[str, Any]:
        """
        Safely query the RAG agent with comprehensive error handling.

        Args:
            query: The user's query string
            conversation_id: Optional conversation ID for context continuity
            debug: Whether to enable debug mode

        Returns:
            Dict containing the agent's response and metadata, or error information
        """
        try:
            agent = self.initialize_agent()

            # Validate input parameters
            if not query or not query.strip():
                raise ValueError("Query cannot be empty or whitespace only")

            if len(query.strip()) < 3:
                raise ValueError("Query must be at least 3 characters long")

            # Call the agent with the provided query and parameters
            # Use the new API that returns both answer and sources
            response_data = agent.ask_with_sources(query.strip(), conversation_id=conversation_id, debug=debug)

            # Return the response in the expected format
            result = {
                "response": response_data["answer"],
                "conversation_id": conversation_id,  # May be updated by the agent
                "sources": response_data["sources"]
            }

            # If debug mode was enabled, include debug information
            if debug:
                result["debug_info"] = {
                    "processing_info": "Debug mode enabled - additional information available internally",
                    "retrieved_sources_count": len(response_data["sources"])
                }

            return result

        except ValueError as ve:
            # Handle validation errors specifically
            self.logger.warning(f"Validation error in query_agent: {str(ve)}")
            return {
                "response": "Invalid query provided. Please ensure your query is at least 3 characters long.",
                "error": "validation_error",
                "error_details": str(ve)
            }

        except Exception as e:
            # Handle all other errors
            self.logger.error(f"Error in safe_query_agent: {str(e)}")
            return {
                "response": "An error occurred while processing your request. Please try again later.",
                "error": "internal_error",
                "error_details": str(e)
            }

    def query_agent(self, query: str, conversation_id: Optional[str] = None, debug: bool = False) -> Dict[str, Any]:
        """
        Query the RAG agent and return the response.

        Args:
            query: The user's query string
            conversation_id: Optional conversation ID for context continuity
            debug: Whether to enable debug mode

        Returns:
            Dict containing the agent's response and metadata
        """
        try:
            agent = self.initialize_agent()

            # Call the agent with the provided query and parameters
            # Use the new API that returns both answer and sources
            response_data = agent.ask_with_sources(query, conversation_id=conversation_id, debug=debug)

            # Return the response in the expected format
            result = {
                "response": response_data["answer"],
                "conversation_id": conversation_id,  # May be updated by the agent
                "sources": response_data["sources"]
            }

            # If debug mode was enabled, include debug information
            if debug:
                result["debug_info"] = {
                    "retrieved_sources_count": len(response_data["sources"]),
                    "processing_info": "Debug mode enabled - retrieved sources info available internally"
                }

            return result

        except Exception as e:
            self.logger.error(f"Error querying agent: {str(e)}")
            raise

    def get_content_status(self) -> Dict[str, Any]:
        """
        Get the status of the content in the Qdrant collection.

        Returns:
            Dict containing collection status information
        """
        try:
            agent = self.initialize_agent()
            return agent.get_content_status()
        except Exception as e:
            self.logger.error(f"Error getting content status: {str(e)}")
            return {
                "collection_name": "unknown",
                "total_documents": 0,
                "status": "error",
                "error": str(e)
            }


# Global agent manager instance for use in the API
agent_manager = AgentManager()