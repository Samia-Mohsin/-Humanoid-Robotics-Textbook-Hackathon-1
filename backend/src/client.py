import openai
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

class OpenAIClient:
    """
    OpenAI client using Chat Completions API instead of deprecated Assistants v1 API
    """

    def __init__(self):
        # Initialize OpenAI client with API key from environment
        self.client = openai.OpenAI(
            api_key=os.getenv("OPENAI_API_KEY")
        )
        # Use a default model, but this can be overridden
        self.model = os.getenv("AGENT_MODEL", "gpt-3.5-turbo")

    def create_completion(self, messages, model=None, temperature=0.7, max_tokens=1000):
        """
        Create a completion using the Chat Completions API

        Args:
            messages: List of message dictionaries with 'role' and 'content'
            model: Optional model override
            temperature: Controls randomness (0.0 to 1.0)
            max_tokens: Maximum tokens to generate

        Returns:
            OpenAI API response object
        """
        model_to_use = model or self.model

        response = self.client.chat.completions.create(
            model=model_to_use,
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens
        )

        return response

# Create a singleton instance
openai_client = OpenAIClient()