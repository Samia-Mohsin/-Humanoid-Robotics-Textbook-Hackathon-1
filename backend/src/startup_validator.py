import os
import re
from typing import List, Dict
import importlib.util
import sys

class StartupValidator:
    """
    Validates the application at startup to detect deprecated API usage
    """

    def __init__(self):
        # Define patterns for deprecated OpenAI Assistants v1 API calls
        self.deprecated_patterns = [
            r'openai_client\.beta\.threads\.create\(',
            r'openai_client\.beta\.threads\.messages\.create\(',
            r'openai_client\.beta\.assistants\.create\(',
            r'openai_client\.beta\.threads\.runs\.create\(',
            r'openai_client\.beta\.threads\.runs\.retrieve\(',
            r'openai_client\.beta\.threads\.messages\.list\(',
            r'openai\.OpenAI\(\s*\)\.beta\.threads',
            r'openai\.OpenAI\(\s*\)\.beta\.assistants',
            r'openai\.OpenAI\(\s*\)\.beta\.runs',
            r'openai\.AzureOpenAI\(\s*\)\.beta\.threads',
            r'openai\.AzureOpenAI\(\s*\)\.beta\.assistants',
            r'openai\.AzureOpenAI\(\s*\)\.beta\.runs',
        ]

    def scan_file_for_deprecated_api(self, file_path: str) -> List[Dict[str, any]]:
        """
        Scans a file for deprecated API usage

        Args:
            file_path: Path to the file to scan

        Returns:
            List of matches with line numbers and content
        """
        matches = []
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                lines = file.readlines()
                for line_num, line in enumerate(lines, start=1):
                    for pattern in self.deprecated_patterns:
                        if re.search(pattern, line):
                            matches.append({
                                'file': file_path,
                                'line': line_num,
                                'content': line.strip(),
                                'pattern': pattern
                            })
        except Exception:
            # If we can't read the file, skip it
            pass

        return matches

    def scan_directory(self, directory: str, extensions: List[str] = ['.py']) -> List[Dict[str, any]]:
        """
        Scans a directory for deprecated API usage

        Args:
            directory: Directory path to scan
            extensions: File extensions to scan

        Returns:
            List of matches found in the directory
        """
        import os
        matches = []

        for root, dirs, files in os.walk(directory):
            # Skip certain directories that don't contain application code
            dirs[:] = [d for d in dirs if d not in ['.git', '__pycache__', 'node_modules', '.venv', 'venv']]

            for file in files:
                if any(file.endswith(ext) for ext in extensions):
                    file_path = os.path.join(root, file)
                    matches.extend(self.scan_file_for_deprecated_api(file_path))

        return matches

    def validate_deprecated_api_usage(self, scan_paths: List[str] = None) -> Dict[str, any]:
        """
        Validates that no deprecated API usage exists in the codebase

        Args:
            scan_paths: List of paths to scan (defaults to common application paths)

        Returns:
            Dictionary with validation results
        """
        if scan_paths is None:
            scan_paths = [
                './agent.py',
                './agent_integration.py',
                './api.py',
                './backend/',
                './models.py'
            ]

        all_matches = []
        for path in scan_paths:
            if os.path.exists(path):
                if os.path.isfile(path):
                    all_matches.extend(self.scan_file_for_deprecated_api(path))
                elif os.path.isdir(path):
                    all_matches.extend(self.scan_directory(path))

        return {
            'deprecated_api_found': len(all_matches) > 0,
            'matches': all_matches,
            'count': len(all_matches)
        }

    def run_startup_validation(self) -> bool:
        """
        Runs the full startup validation and fails if deprecated APIs are found

        Returns:
            True if validation passes, raises exception if deprecated APIs found
        """
        result = self.validate_deprecated_api_usage()

        if result['deprecated_api_found']:
            print(f"❌ STARTUP VALIDATION FAILED: Found {result['count']} deprecated API usage(s)")
            for match in result['matches']:
                print(f"  - {match['file']}:{match['line']} - {match['content']}")
            print("\nPlease update the code to use the modern OpenAI Chat Completions API instead of deprecated Assistants v1 API.")
            raise RuntimeError(f"Startup validation failed: {result['count']} deprecated API usage(s) detected")
        else:
            print("✅ Startup validation passed: No deprecated API usage detected")
            return True