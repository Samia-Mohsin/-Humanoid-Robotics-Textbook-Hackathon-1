# Quickstart: AI Agent with Retrieval-Augmented Capabilities

## Setup

1. **Install Dependencies**
   ```bash
   pip install openai qdrant-client python-dotenv
   ```

2. **Configure Environment**
   Create a `.env` file in the root directory with:
   ```
   OPENAI_API_KEY=your_openai_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=your_collection_name
   ```

## Usage

### Run the AI Agent

```bash
python agent.py
```

### Interactive Mode
The agent starts in interactive mode where you can ask questions about humanoid robotics:

```
Humanoid Robotics Agent ready. Ask me anything about humanoid robotics!
> What is ROS 2?
[Agent responds based on retrieved content from the textbook]
> Can you explain more about navigation?
[Agent maintains context and provides follow-up response]
```

### Programmatic Usage
You can also use the agent programmatically:

```python
from agent import HumanoidRoboticsAgent

agent = HumanoidRoboticsAgent()
response = agent.ask("What is ROS 2?")
print(response)
```

## Expected Output
The agent will:
- Connect to Qdrant and search for relevant content
- Retrieve text chunks from the humanoid robotics textbook
- Generate responses based only on the retrieved content
- Maintain conversation context for follow-up questions

## Troubleshooting

- If you get connection errors, verify your OpenAI and Qdrant API keys in `.env`
- If no relevant results are returned, check that the Qdrant collection contains data
- If responses seem unrelated, verify that the retrieval pipeline is working correctly
- For context issues, check that conversation history is being maintained properly