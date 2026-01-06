# Data Model: AI Agent with Retrieval-Augmented Capabilities

## Core Entities

### Agent
- **Description**: AI entity created using OpenAI Agents SDK that orchestrates the RAG workflow
- **Fields**:
  - `agent_id`: string - Unique identifier for the agent instance
  - `name`: string - Name of the agent
  - `description`: string - Description of the agent's purpose
  - `tools`: list of Tool - List of tools available to the agent
  - `model`: string - AI model used by the agent (e.g., gpt-4, gpt-3.5-turbo)

### Tool
- **Description**: Tool component that connects to Qdrant and retrieves relevant text chunks based on user queries
- **Fields**:
  - `tool_id`: string - Unique identifier for the tool
  - `name`: string - Name of the tool
  - `description`: string - Description of what the tool does
  - `function`: callable - Function that implements the tool's functionality
  - `parameters`: dict - Parameters required by the tool

### ContentChunk
- **Description**: Retrieved text segments from the humanoid robotics textbook with associated metadata
- **Fields**:
  - `chunk_id`: string - Unique identifier for the content chunk
  - `text`: string - The actual text content of the chunk
  - `source_url`: string - URL of the original source document
  - `metadata`: dict - Additional metadata including document ID and other info
  - `relevance_score`: float - Relevance score for the chunk in relation to the query

### ConversationContext
- **Description**: State management for multi-turn interactions between user and agent
- **Fields**:
  - `conversation_id`: string - Unique identifier for the conversation
  - `messages`: list of Message - List of messages in the conversation
  - `created_at`: datetime - When the conversation was started
  - `last_updated`: datetime - When the conversation was last updated
  - `context_window`: list of string - Recent messages for context

### Message
- **Description**: Individual message in a conversation between user and agent
- **Fields**:
  - `message_id`: string - Unique identifier for the message
  - `role`: string - Role of the message sender (user, assistant, system)
  - `content`: string - Content of the message
  - `timestamp`: datetime - When the message was created
  - `retrieved_chunks`: list of ContentChunk - Chunks used to generate the response

## Relationships
- An `Agent` has multiple `Tool` instances
- A `Tool` retrieves multiple `ContentChunk` items
- A `ConversationContext` contains multiple `Message` instances
- A `Message` may reference multiple `ContentChunk` items used for response generation

## Validation Rules
- `Agent.agent_id` must be unique
- `Tool.parameters` must match the expected function signature
- `ContentChunk.text` must not be empty
- `ContentChunk.relevance_score` must be between 0 and 1
- `Message.role` must be one of: "user", "assistant", "system"
- `ConversationContext.messages` must not exceed maximum length