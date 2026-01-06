#!/usr/bin/env python3
"""
Document Ingestion Script for Humanoid Robotics Textbook

This script ingests MD/MDX files from the /docs folder into Qdrant for the RAG system.
It processes documents, chunks them appropriately, generates embeddings, and stores them in Qdrant.
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
import cohere
import tiktoken
try:
    import frontmatter  # For parsing YAML frontmatter
except ImportError:
    frontmatter = None

# Load environment variables
load_dotenv()

class TextbookIngestor:
    """Handles the ingestion of textbook content into Qdrant for RAG system"""

    def __init__(self,
                 qdrant_url: str = None,
                 qdrant_api_key: str = None,
                 cohere_api_key: str = None,
                 collection_name: str = "humanoid_robotics_docs"):
        self.qdrant_url = qdrant_url or os.getenv("QDRANT_URL")
        self.qdrant_api_key = qdrant_api_key or os.getenv("QDRANT_API_KEY")
        self.cohere_api_key = cohere_api_key or os.getenv("COHERE_API_KEY")
        self.collection_name = collection_name

        # Initialize clients
        self.qdrant_client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            timeout=30
        )

        self.cohere_client = cohere.Client(self.cohere_api_key)

        # Verify connections
        try:
            self.qdrant_client.get_collection(self.collection_name)
            print(f"Connected to existing Qdrant collection: {self.collection_name}")
        except:
            # Create collection if it doesn't exist
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),  # Updated to match hybrid embeddings
            )
            print(f"Created new Qdrant collection: {self.collection_name}")

        # Verify Cohere connection
        try:
            self.cohere_client.embed(
                texts=["test"],
                model="embed-english-v3.0",
                input_type="search_document"
            )
            print("Connected to Cohere API")
        except Exception as e:
            # For trial keys, we might hit rate limits during initialization
            # We'll handle this in the actual embedding calls
            print(f"Note: Cohere connection test failed (possibly due to rate limits): {str(e)}")
            print("This is expected with trial keys - proceeding anyway...")

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

    def extract_module_info(self, path_parts: tuple) -> Dict[str, str]:
        """Extract module and section information from file path"""
        # Look for patterns like 'module1', 'module2', etc. in the path
        module_info = {"module": "", "section": ""}

        for i, part in enumerate(path_parts):
            # Check if this part contains a module pattern
            if 'module' in part.lower():
                # Extract module name (e.g., 'module1', 'module-1', 'Module1')
                module_match = re.search(r'(module[\w-]*\d+)', part, re.IGNORECASE)
                if module_match:
                    module_info["module"] = module_match.group(1)

            # The section could be the directory name or the file name
            if i == len(path_parts) - 1:  # This is the file name
                section_name = Path(part).stem  # Remove file extension
                module_info["section"] = section_name.replace('-', ' ').replace('_', ' ').title()
            elif 'module' not in part.lower() and i > 0:
                # This might be a section/directory name within a module
                if 'module' in path_parts[i-1].lower() or module_info["module"]:
                    module_info["section"] = part.replace('-', ' ').replace('_', ' ').title()

        return module_info

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

        # Extract frontmatter if present and frontmatter module is available
        metadata = {}
        if frontmatter:
            try:
                post = frontmatter.loads(content)
                content = post.content
                metadata = post.metadata
            except:
                # If frontmatter parsing fails, continue with original content and empty metadata
                pass

        # Clean content
        cleaned_content = self.clean_markdown_content(content)

        # Determine module and section from file path
        path_parts = Path(file_path).parts
        module_info = self.extract_module_info(path_parts)

        # Chunk the content
        chunks = self.chunk_text_by_headings(cleaned_content, max_tokens=600, overlap_tokens=150)

        # Import the hybrid embedding function
        from hybrid_embeddings import generate_embedding_with_hybrid_fallback

        # Create chunk records with metadata
        chunk_records = []
        for i, chunk in enumerate(chunks):
            # Generate embedding for the chunk using hybrid fallback
            try:
                embedding = generate_embedding_with_hybrid_fallback(
                    chunk['text'],
                    cohere_client=self.cohere_client,  # Will use Cohere first, then local fallback
                    cohere_model="embed-english-v3.0"
                )
            except Exception as e:
                print(f"Error generating embedding for chunk: {e}")
                continue  # Skip this chunk if embedding fails

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

            # Add any frontmatter metadata
            chunk_metadata.update(metadata)

            chunk_record = {
                "id": str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{file_path}_{i}".encode('utf-8'))),
                "text": chunk['text'],
                "vector": embedding,
                "metadata": chunk_metadata
            }

            chunk_records.append(chunk_record)

        print(f"Processed {len(chunk_records)} chunks from {file_path}")
        return chunk_records


    def ingest_documents(self, docs_folder: str = "docs") -> int:
        """Ingest all markdown files from the docs folder into Qdrant"""
        print(f"Starting ingestion from {docs_folder}/ folder...")

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
    """Main function to run the ingestion process"""
    print("Starting Humanoid Robotics Textbook Ingestion Process...")

    try:
        ingestor = TextbookIngestor()

        # Process all files in the frontend-book/docs folder
        total_chunks = ingestor.ingest_documents(docs_folder="frontend-book/docs")

        print(f"\nIngestion completed successfully!")
        print(f"Total chunks ingested: {total_chunks}")
        print(f"Stored in Qdrant collection: {ingestor.collection_name}")

        return 0

    except Exception as e:
        print(f"Ingestion failed: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())