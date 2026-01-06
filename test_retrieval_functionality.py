#!/usr/bin/env python3
"""
Test script to validate retrieval functionality with core technical queries
"""
import os
from dotenv import load_dotenv
from agent import HumanoidRoboticsAgent

# Load environment variables
load_dotenv()

def test_retrieval_functionality():
    """Test retrieval functionality with core technical queries"""
    print("Testing retrieval functionality with core technical queries...\n")

    try:
        # Initialize agent
        print("Initializing agent...")
        agent = HumanoidRoboticsAgent()
        print("Agent initialized successfully!\n")

        # Core technical queries to test
        test_queries = [
            "What is ROS 2?",
            "Explain nodes and topics in ROS 2",
            "What is URDF?",
            "How does Gazebo work?",
            "What is NVIDIA Isaac Sim?"
        ]

        for i, query in enumerate(test_queries, 1):
            print(f"Test {i}: Query: '{query}'")
            print("-" * 50)

            try:
                # Get response with debug mode enabled
                response = agent.ask(query, debug=True)
                print(f"Response: {response}")
            except Exception as e:
                print(f"Error processing query '{query}': {str(e)}")

            print("\n" + "="*80 + "\n")

        # Test with a general query to ensure basic functionality
        print("Final test: General question about humanoid robotics")
        print("-" * 50)
        general_response = agent.ask("What is the main goal of humanoid robotics?", debug=True)
        print(f"Response: {general_response}")

        print("\nAll tests completed successfully!")
        return True

    except Exception as e:
        print(f"Error during testing: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_retrieval_functionality()
    if success:
        print("\nSUCCESS: All retrieval functionality tests passed!")
        exit(0)
    else:
        print("\nERROR: Some retrieval functionality tests failed!")
        exit(1)