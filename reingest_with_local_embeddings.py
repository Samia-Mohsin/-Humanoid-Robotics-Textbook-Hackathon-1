#!/usr/bin/env python3
"""
Re-ingest the textbook content using local embeddings to fix the Cohere API dependency issue.

This script re-ingests the documentation using local embeddings that match the required dimension,
allowing the RAG system to work without depending on the Cohere API.
"""
import os
import re
import glob
import uuid
from pathlib import Path
from typing import List, Dict, Any
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer
import tiktoken

# Load environment variables
load_dotenv()

class LocalTextbookIngestor:
    """Handles the ingestion of textbook content into Qdrant using local embeddings"""

    def __init__(self,
                 qdrant_url: str = None,
                 qdrant_api_key: str = None,
                 collection_name: str = "humanoid_robotics_docs"):
        self.qdrant_url = qdrant_url or os.getenv("QDRANT_URL")
        self.qdrant_api_key = qdrant_api_key or os.getenv("QDRANT_API_KEY")
        self.collection_name = collection_name

        # Initialize clients
        self.qdrant_client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            timeout=30
        )

        # Initialize local embedding model that produces 768-dim embeddings
        # (We'll use all-mpnet-base-v2 which produces 768-dim embeddings)
        print("Loading local embedding model (this may take a moment)...")
        self.embedding_model = SentenceTransformer('all-mpnet-base-v2')
        print("Local embedding model loaded successfully")

        # Get embedding dimension
        sample_embedding = self.embedding_model.encode(["test"])
        self.embedding_dimension = len(sample_embedding[0])
        print(f"Model produces {self.embedding_dimension}-dimensional embeddings")

        # Check if collection exists and create/recreate it with correct dimensions
        try:
            self.qdrant_client.get_collection(self.collection_name)
            print(f"Collection {self.collection_name} exists, recreating with new dimensions...")
            self.qdrant_client.delete_collection(self.collection_name)
        except:
            pass  # Collection doesn't exist, will create it

        # Create collection with the correct vector size
        self.qdrant_client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(size=self.embedding_dimension, distance=models.Distance.COSINE),
        )
        print(f"Created new Qdrant collection: {self.collection_name} with {self.embedding_dimension}-dimensional vectors")

        # Initialize tokenizer for chunking
        self.tokenizer = tiktoken.get_encoding("cl100k_base")  # Good for most models

    def num_tokens_from_text(self, text: str) -> int:
        """Return the number of tokens in a text string"""
        return len(self.tokenizer.encode(text))

    def clean_markdown_content(self, content: str) -> str:
        """Clean markdown content by removing frontmatter and code fences"""
        # Remove YAML frontmatter if present
        content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

        # Remove Docusaurus-specific syntax (keep content)
        content = re.sub(r'{%.*?%}', '', content)  # Remove liquid-style tags
        content = re.sub(r'<.*?>', '', content)    # Remove HTML tags (simple)

        # Clean up admonitions and tabs
        content = re.sub(r':::(\w+)\n(.*?)\n:::\n', r'\2\n', content, flags=re.DOTALL)

        # Normalize whitespace
        content = re.sub(r'\n\s*\n', '\n\n', content)  # Remove extra blank lines
        content = content.strip()

        return content

    def chunk_text_by_headings(self, text: str, max_tokens: int = 600, overlap_tokens: int = 150) -> List[Dict[str, Any]]:
        """Chunk text by headings, keeping logical sections together"""
        chunks = []

        # Split by headings (###, ##, #)
        heading_pattern = r'(\n#{1,3}\s+[^\n]+\n?)'
        parts = re.split(heading_pattern, text)

        # Combine headings with their content
        combined_parts = []
        i = 0
        while i < len(parts):
            if parts[i].startswith('\n#'):  # This is a heading
                heading = parts[i]
                content = ""
                if i + 1 < len(parts):
                    content = parts[i + 1]
                combined_parts.append(heading + content)
                i += 2
            else:
                # Standalone content without heading
                if parts[i].strip():
                    combined_parts.append(parts[i])
                i += 1

        # Process each part
        for part in combined_parts:
            if not part.strip():
                continue

            # If the part is too large, chunk it by paragraphs
            if self.num_tokens_from_text(part) > max_tokens:
                sub_chunks = self.chunk_by_paragraphs(part, max_tokens, overlap_tokens)
                chunks.extend(sub_chunks)
            else:
                chunks.append({
                    'text': part.strip()
                })

        # Apply overlap between chunks
        overlapping_chunks = []
        for i, chunk in enumerate(chunks):
            chunk_text = chunk['text']

            # Add overlap from previous chunk if available and not the first chunk
            if i > 0 and overlapping_chunks:
                prev_chunk = overlapping_chunks[-1]['text']

                # Create overlap by taking the last portion of the previous chunk
                prev_tokens = self.tokenizer.encode(prev_chunk)
                overlap_size = min(len(prev_tokens), overlap_tokens)
                if overlap_size > 0:
                    prev_overlap_tokens = prev_tokens[-overlap_size:]
                    prev_overlap_text = self.tokenizer.decode(prev_overlap_tokens)
                    chunk_text = prev_overlap_text + "\n\n" + chunk_text

            overlapping_chunks.append({
                'text': chunk_text
            })

        return overlapping_chunks

    def chunk_by_paragraphs(self, text: str, max_tokens: int, overlap_tokens: int) -> List[Dict[str, Any]]:
        """Chunk text by paragraphs when it's too large"""
        chunks = []

        # Split on paragraphs (double newlines)
        paragraphs = text.split('\n\n')

        current_chunk = ""
        for paragraph in paragraphs:
            test_chunk = current_chunk + "\n\n" + paragraph if current_chunk else paragraph

            if self.num_tokens_from_text(test_chunk) <= max_tokens:
                current_chunk = test_chunk
            else:
                # If current chunk has content, save it
                if current_chunk.strip():
                    chunks.append({'text': current_chunk.strip()})

                # If the single paragraph is too large, split it by sentences
                if self.num_tokens_from_text(paragraph) > max_tokens:
                    sentence_chunks = self.chunk_by_sentences(paragraph, max_tokens, overlap_tokens)
                    chunks.extend(sentence_chunks)
                else:
                    current_chunk = paragraph

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append({'text': current_chunk.strip()})

        return chunks

    def chunk_by_sentences(self, text: str, max_tokens: int, overlap_tokens: int) -> List[Dict[str, Any]]:
        """Chunk text by sentences when paragraphs are still too large"""
        import re

        # Split into sentences
        sentences = re.split(r'(?<=[.!?])\s+', text)

        chunks = []
        current_chunk = ""

        for sentence in sentences:
            test_chunk = current_chunk + " " + sentence if current_chunk else sentence

            if self.num_tokens_from_text(test_chunk) <= max_tokens:
                current_chunk = test_chunk
            else:
                if current_chunk.strip():  # Save the current chunk if it has content
                    chunks.append({'text': current_chunk.strip()})

                    # Add overlap by taking the last few tokens from the current chunk
                    if overlap_tokens > 0:
                        chunk_tokens = self.tokenizer.encode(current_chunk)
                        overlap_size = min(len(chunk_tokens), overlap_tokens)
                        if overlap_size > 0:
                            overlap_tokens_list = chunk_tokens[-overlap_size:]
                            overlap_text = self.tokenizer.decode(overlap_tokens_list)
                            current_chunk = overlap_text + " " + sentence
                    else:
                        current_chunk = sentence
                else:
                    # If a single sentence is too long, force split it by tokens
                    forced_chunks = self.force_chunk_by_tokens(sentence, max_tokens, overlap_tokens)
                    chunks.extend(forced_chunks)
                    current_chunk = ""  # Reset after forced chunks

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append({'text': current_chunk.strip()})

        return chunks

    def force_chunk_by_tokens(self, text: str, max_tokens: int, overlap_tokens: int) -> List[Dict[str, Any]]:
        """Force chunk text by token count when other methods fail"""
        tokens = self.tokenizer.encode(text)
        chunks = []

        start_idx = 0
        while start_idx < len(tokens):
            end_idx = start_idx + max_tokens
            if end_idx > len(tokens):
                end_idx = len(tokens)

            chunk_tokens = tokens[start_idx:end_idx]
            chunk_text = self.tokenizer.decode(chunk_tokens).strip()

            if chunk_text:  # Only add non-empty chunks
                chunks.append({'text': chunk_text})

            # Move start index forward, considering overlap
            if overlap_tokens > 0 and end_idx < len(tokens):
                start_idx = end_idx - overlap_tokens
            else:
                start_idx = end_idx

        return chunks

    def process_file(self, file_path: str) -> List[Dict[str, Any]]:
        """Process a single markdown file and return chunks with metadata"""
        print(f"Processing file: {file_path}")

        # Read file content
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Clean content
        cleaned_content = self.clean_markdown_content(content)

        # Determine module and section from file path
        path_parts = Path(file_path).parts
        # Extract module info based on path structure
        module_info = {"module": "", "section": ""}
        for part in path_parts:
            if 'module' in part.lower():
                module_info["module"] = part
            elif part.endswith('.md') or part.endswith('.mdx'):
                module_info["section"] = part.replace('.md', '').replace('.mdx', '').replace('-', ' ').replace('_', ' ').title()

        # Chunk the content
        chunks = self.chunk_text_by_headings(cleaned_content, max_tokens=600, overlap_tokens=150)

        # Create chunk records with metadata
        chunk_records = []
        for i, chunk in enumerate(chunks):
            # Generate embedding for the chunk using local model
            try:
                embedding = self.embedding_model.encode([chunk['text']])[0].tolist()
            except Exception as e:
                print(f"Error generating local embedding for chunk: {e}")
                continue

            # Create metadata
            chunk_metadata = {
                "source_url": f"file://{os.path.abspath(file_path)}",
                "module": module_info.get("module", ""),
                "section": module_info.get("section", ""),
                "file_path": str(Path(file_path).absolute()),
                "chunk_index": i,
                "total_chunks": len(chunks),
                "original_file_name": Path(file_path).name
            }

            chunk_record = {
                "id": str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{file_path}_{i}".encode('utf-8'))),
                "text": chunk['text'],
                "vector": embedding,
                "metadata": chunk_metadata
            }

            chunk_records.append(chunk_record)

        print(f"Processed {len(chunk_records)} chunks from {file_path}")
        return chunk_records

    def ingest_documents(self, docs_folder: str = "frontend-book/docs") -> int:
        """Ingest all markdown files from the docs folder into Qdrant"""
        print(f"Starting re-ingestion from {docs_folder}/ folder...")

        # Find all MD and MDX files
        md_files = glob.glob(f"{docs_folder}/**/*.md", recursive=True)
        mdx_files = glob.glob(f"{docs_folder}/**/*.mdx", recursive=True)
        all_files = md_files + mdx_files

        print(f"Found {len(all_files)} files to process")

        all_chunk_records = []
        for file_path in all_files:
            try:
                chunk_records = self.process_file(file_path)
                all_chunk_records.extend(chunk_records)
            except Exception as e:
                print(f"Error processing file {file_path}: {e}")
                import traceback
                traceback.print_exc()
                continue

        # Upload chunks to Qdrant
        if all_chunk_records:
            print(f"Uploading {len(all_chunk_records)} chunks to Qdrant...")

            points = []
            for record in all_chunk_records:
                point = models.PointStruct(
                    id=record["id"],
                    vector=record["vector"],
                    payload={
                        "text": record["text"],
                        "metadata": record["metadata"],
                        "source_url": record["metadata"]["source_url"],
                        "module": record["metadata"]["module"],
                        "section": record["metadata"]["section"]
                    }
                )
                points.append(point)

            # Upsert points to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            print(f"Successfully uploaded {len(all_chunk_records)} chunks to Qdrant collection '{self.collection_name}'")
        else:
            print("No chunks to upload - check if files were processed successfully")

        return len(all_chunk_records)


def main():
    """Main function to run the re-ingestion process"""
    print("Starting Re-Ingestion Process with Local Embeddings...")
    print("This will recreate the vector database using local embeddings to avoid Cohere API dependencies.")
    print("=" * 60)

    try:
        ingestor = LocalTextbookIngestor()

        # Process all files in the frontend-book/docs folder
        total_chunks = ingestor.ingest_documents(docs_folder="frontend-book/docs")

        print(f"\nRe-ingestion completed successfully!")
        print(f"Total chunks ingested: {total_chunks}")
        print(f"Stored in Qdrant collection: {ingestor.collection_name}")
        print(f"Embedding dimension: {ingestor.embedding_dimension}")
        print("The RAG system can now work without Cohere API dependency!")

        return 0

    except Exception as e:
        print(f"Re-ingestion failed: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())