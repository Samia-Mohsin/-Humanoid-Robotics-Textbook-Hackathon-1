"""
Text chunking functionality for the URL Ingestion & Embedding Pipeline
"""
import re
from typing import List, Tuple
from models import DocumentChunk


def split_by_sentences(text: str) -> List[str]:
    """
    Split text by sentences while maintaining context.

    Args:
        text: Input text to split

    Returns:
        List of sentence chunks
    """
    # Split by sentence endings (., !, ?, etc.) but keep the punctuation
    sentences = re.split(r'(?<=[.!?])\s+', text)
    # Filter out empty strings and strip whitespace
    sentences = [s.strip() for s in sentences if s.strip()]
    return sentences


def calculate_token_count(text: str) -> int:
    """
    Calculate approximate token count for text.
    Using a simple approach: count words as tokens (more accurate would use tiktoken).

    Args:
        text: Input text

    Returns:
        Approximate token count
    """
    # Simple word-based token estimation (1 word ≈ 1 token on average)
    if not text:
        return 0
    words = re.findall(r'\b\w+\b', text)
    return len(words)


def implement_text_chunking(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
    """
    Implement text chunking function with specified chunk size and overlap.

    Args:
        text: Input text to chunk
        chunk_size: Target size for each chunk (in tokens)
        overlap: Overlap between chunks (in tokens)

    Returns:
        List of text chunks
    """
    if not text:
        return []

    # First, split into sentences to maintain semantic boundaries
    sentences = split_by_sentences(text)

    if not sentences:
        # If no sentences found, fall back to chunking by character count
        chunks = []
        start = 0
        text_tokens = calculate_token_count(text)

        while start < len(text):
            # Determine the end position
            end = start
            current_tokens = 0

            # Move end pointer while we're under the chunk size
            while end < len(text) and current_tokens < chunk_size:
                segment = text[start:end+1]
                segment_tokens = calculate_token_count(segment)

                if segment_tokens <= chunk_size:
                    end += 1
                    current_tokens = segment_tokens
                else:
                    break

            # Add the chunk
            chunk = text[start:end].strip()
            if chunk:
                chunks.append(chunk)

            # Move start to create overlap
            if start == end:  # If we couldn't advance, move by one character to avoid infinite loop
                start = end + 1
            else:
                # Calculate overlap by moving start back
                overlap_start = start
                while overlap_start < end:
                    overlap_segment = text[overlap_start:end]
                    overlap_tokens = calculate_token_count(overlap_segment)

                    if overlap_tokens <= overlap:
                        start = overlap_start
                        break
                    overlap_start += 1
                else:
                    start = end  # If we can't create proper overlap, just move to end

        return chunks

    # Process sentences into chunks
    chunks = []
    current_chunk = ""
    current_tokens = 0

    for sentence in sentences:
        sentence_tokens = calculate_token_count(sentence)

        # If adding this sentence would exceed chunk size
        if current_tokens + sentence_tokens > chunk_size and current_chunk:
            # Save the current chunk
            chunks.append(current_chunk.strip())

            # Handle overlap
            if overlap > 0:
                # Find sentences to include in overlap
                overlap_chunk = ""
                overlap_tokens = 0
                temp_sentences = []

                # Work backwards from the current sentence to accumulate overlap
                temp_sentence_tokens = sentence_tokens
                if temp_sentence_tokens <= overlap:
                    overlap_chunk = sentence
                    overlap_tokens = temp_sentence_tokens

                # Add the current sentence to the new chunk
                current_chunk = sentence
                current_tokens = sentence_tokens
            else:
                # No overlap, start fresh
                current_chunk = sentence
                current_tokens = sentence_tokens
        else:
            # Add sentence to current chunk
            if current_chunk:
                current_chunk += " " + sentence
            else:
                current_chunk = sentence
            current_tokens += sentence_tokens

    # Add the final chunk if it has content
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def add_chunk_overlap_logic(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
    """
    Add chunk overlap logic to maintain context between chunks.

    Args:
        text: Input text to chunk with overlap
        chunk_size: Target size for each chunk (in tokens)
        overlap: Overlap between chunks (in tokens)

    Returns:
        List of text chunks with proper overlap
    """
    return implement_text_chunking(text, chunk_size, overlap)


def split_text_by_sentences(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
    """
    Create function to split text by sentences while maintaining chunk size.

    Args:
        text: Input text to split
        chunk_size: Target size for each chunk (in tokens)
        overlap: Overlap between chunks (in tokens)

    Returns:
        List of text chunks split by sentences
    """
    # This is a wrapper function that uses the main chunking logic
    return implement_text_chunking(text, chunk_size, overlap)


def chunk_document(document_chunk: DocumentChunk, chunk_size: int = 512, overlap: int = 50) -> List[DocumentChunk]:
    """
    Chunk a DocumentChunk into multiple smaller DocumentChunks.

    Args:
        document_chunk: The original document chunk to split
        chunk_size: Target size for each chunk (in tokens)
        overlap: Overlap between chunks (in tokens)

    Returns:
        List of smaller DocumentChunks
    """
    text_chunks = implement_text_chunking(document_chunk.content, chunk_size, overlap)
    result_chunks = []

    for i, text in enumerate(text_chunks):
        # Create a new DocumentChunk for each text chunk
        # Preserve source info and add position
        new_chunk = DocumentChunk(
            chunk_id=f"{document_chunk.chunk_id}_chunk_{i}",
            content=text,
            source_url=document_chunk.source_url,
            title=document_chunk.title,
            section=document_chunk.section,
            position=i,
            metadata={**document_chunk.metadata, "original_chunk_id": document_chunk.chunk_id}
        )
        result_chunks.append(new_chunk)

    return result_chunks