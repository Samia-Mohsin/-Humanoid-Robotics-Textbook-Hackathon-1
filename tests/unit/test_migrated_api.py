"""
Unit tests for the migrated OpenAI API integration.

Tests the new Chat Completions API implementation instead of deprecated Assistants v1 API.
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
import os
from agent import HumanoidRoboticsAgent
from agent_integration import AgentManager
from backend.src.client import OpenAIClient
from backend.src.rag_service import RAGService
from backend.src.error_handler import APIErrorHandler
from backend.src.startup_validator import StartupValidator


class TestOpenAIClient:
    """Test the new OpenAI client using Chat Completions API"""

    def test_client_initialization(self):
        """Test that OpenAI client is initialized correctly"""
        client = OpenAIClient()
        assert client.client is not None
        assert hasattr(client, 'create_completion')

    @patch('backend.src.client.openai.OpenAI')
    def test_create_completion_call(self, mock_openai_client):
        """Test that create_completion calls the Chat Completions API"""
        # Mock the response
        mock_response = Mock()
        mock_choice = Mock()
        mock_choice.message.content = "Test response"
        mock_response.choices = [mock_choice]

        mock_client_instance = Mock()
        mock_client_instance.chat.completions.create.return_value = mock_response
        mock_openai_client.return_value = mock_client_instance

        client = OpenAIClient()
        messages = [{"role": "user", "content": "Hello"}]
        response = client.create_completion(messages)

        # Verify the method was called
        mock_client_instance.chat.completions.create.assert_called_once()
        assert response == mock_response


class TestRAGService:
    """Test the RAG service for content retrieval"""

    def test_rag_service_initialization(self):
        """Test that RAG service initializes with required clients"""
        with patch.dict(os.environ, {
            'QDRANT_URL': 'test_url',
            'QDRANT_API_KEY': 'test_key',
            'QDRANT_COLLECTION_NAME': 'test_collection',
            'COHERE_API_KEY': 'test_cohere_key'
        }):
            rag_service = RAGService()
            assert rag_service.collection_name == 'test_collection'


class TestAgentManager:
    """Test the agent manager with migrated API"""

    def test_agent_manager_initialization(self):
        """Test agent manager initialization"""
        manager = AgentManager()
        assert manager._agent is None
        assert manager._initialized is False

    @patch('agent.HumanoidRoboticsAgent')
    def test_safe_query_agent(self, mock_agent_class):
        """Test the safe query agent method"""
        # Mock the agent
        mock_agent_instance = Mock()
        mock_agent_instance.ask_with_sources.return_value = {
            "answer": "Test answer",
            "sources": ["source1", "source2"]
        }
        mock_agent_class.return_value = mock_agent_instance

        manager = AgentManager()
        # Override the initialize_agent method to return our mock
        manager._agent = mock_agent_instance
        manager._initialized = True

        result = manager.safe_query_agent("test query")

        assert "response" in result
        assert result["response"] == "Test answer"


class TestErrorHandler:
    """Test the error handler functionality"""

    def test_error_logging(self, caplog):
        """Test that errors are logged properly"""
        try:
            raise ValueError("Test error")
        except Exception as e:
            APIErrorHandler.log_error(e, "test_context")

        # Check that the error was logged
        assert "API Error in test_context" in caplog.text


class TestStartupValidator:
    """Test the startup validation functionality"""

    def test_validator_initialization(self):
        """Test that startup validator initializes with deprecated patterns"""
        validator = StartupValidator()
        assert len(validator.deprecated_patterns) > 0
        # Check that it contains some expected patterns
        patterns_str = " ".join(validator.deprecated_patterns)
        assert "beta.threads" in patterns_str
        assert "beta.assistants" in patterns_str