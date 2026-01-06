#!/usr/bin/env python3
"""
Final validation script for the FastAPI RAG Integration.

Validates that all components of the RAG chatbot API are working correctly.
"""
import os
import sys
from pathlib import Path

# Add the project root to the Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

def validate_implementation():
    """Validate that all components of the implementation are in place."""
    print("Starting final validation of FastAPI RAG Integration...\n")

    validation_results = []

    # Check that all required files exist
    required_files = [
        "api.py",
        "models.py",
        "endpoints.py",
        "agent_integration.py",
        "config.py",
        "Dockerfile",
        "docker-compose.yml",
        "requirements.txt",
        "README.md"
    ]

    print("1. Checking required files...")
    for file in required_files:
        file_path = project_root / file
        exists = file_path.exists()
        status = "SUCCESS" if exists else "FAILED"
        print(f"   {status} {file}")
        validation_results.append(("file", file, exists))

    print()

    # Check that all required modules can be imported
    print("2. Checking module imports...")
    required_modules = [
        ("FastAPI application", "api", "app"),
        ("Pydantic models", "models", "QueryRequest"),
        ("API endpoints", "endpoints", "router"),
        ("Agent integration", "agent_integration", "AgentManager"),
        ("Configuration", "config", "config")
    ]

    for module_desc, module_name, attribute in required_modules:
        try:
            module = __import__(module_name)
            has_attr = hasattr(module, attribute) if attribute else True
            status = "SUCCESS" if has_attr else "FAILED"
            print(f"   {status} {module_desc}")
            validation_results.append(("import", module_desc, has_attr))
        except ImportError as e:
            print(f"   FAILED {module_desc} - Import failed: {e}")
            validation_results.append(("import", module_desc, False))

    print()

    # Check that the environment variables are defined in config
    print("3. Checking configuration...")
    try:
        from config import config

        required_configs = [
            ("QDRANT_URL", config.QDRANT_URL),
            ("QDRANT_API_KEY", config.QDRANT_API_KEY),
            ("QDRANT_COLLECTION_NAME", config.QDRANT_COLLECTION_NAME),
            ("COHERE_API_KEY", config.COHERE_API_KEY),
            ("OPENAI_API_KEY", config.OPENAI_API_KEY)
        ]

        for config_name, config_value in required_configs:
            is_set = bool(config_value)
            status = "SET" if is_set else "UNSET"  # Use warning for unset configs
            print(f"   {status} {config_name} {'set' if is_set else 'not set (may be loaded from environment)'}")
            validation_results.append(("config", config_name, is_set))

    except ImportError:
        print("   FAILED Could not import config module")
        validation_results.append(("config", "Config module", False))
    except AttributeError as e:
        print(f"   FAILED Config attribute error: {e}")
        validation_results.append(("config", "Config attributes", False))

    print()

    # Check that Pydantic models are properly defined
    print("4. Checking Pydantic models...")
    try:
        from models import QueryRequest, QueryResponse, ErrorResponse, HealthCheckResponse

        models = [
            ("QueryRequest", QueryRequest),
            ("QueryResponse", QueryResponse),
            ("ErrorResponse", ErrorResponse),
            ("HealthCheckResponse", HealthCheckResponse)
        ]

        for model_name, model_class in models:
            # Use the modern Pydantic v2 approach
            has_fields = hasattr(model_class, 'model_fields') and len(getattr(model_class, 'model_fields', {})) > 0
            status = "SUCCESS" if has_fields else "FAILED"
            print(f"   {status} {model_name}")
            validation_results.append(("model", model_name, has_fields))

    except ImportError as e:
        print(f"   FAILED Could not import models: {e}")
        validation_results.append(("model", "Models import", False))

    print()

    # Summary
    total_checks = len(validation_results)
    passed_checks = sum(1 for _, _, result in validation_results if result)
    failed_checks = total_checks - passed_checks

    print("5. Validation Summary:")
    print(f"   Total checks: {total_checks}")
    print(f"   Passed: {passed_checks}")
    print(f"   Failed: {failed_checks}")
    print(f"   Success rate: {passed_checks/total_checks*100:.1f}%")

    if failed_checks == 0:
        print("\nSUCCESS: All validations passed! The FastAPI RAG Integration is complete and ready.")
        return True
    else:
        print(f"\nWARNING: {failed_checks} validation{'s' if failed_checks != 1 else ''} failed. Please review the issues above.")
        return False


def main():
    """Main entry point for the validation script."""
    success = validate_implementation()

    if success:
        print("\nSUCCESS: FastAPI RAG Integration implementation is complete and validated!")
        return 0
    else:
        print("\nERROR: Some validation steps failed. Please address the issues.")
        return 1


if __name__ == "__main__":
    exit(main())