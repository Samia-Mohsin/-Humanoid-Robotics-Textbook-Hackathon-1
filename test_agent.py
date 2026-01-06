#!/usr/bin/env python3
"""
Test script for the RAG agent with core queries
"""

import os
import sys
from agent import HumanoidRoboticsAgent

def test_agent():
    """Test the agent with core queries"""
    print("Testing the RAG agent with core queries...")

    try:
        # Initialize the agent
        agent = HumanoidRoboticsAgent()
        print("Agent initialized successfully")

        # Define core test queries
        test_queries = [
            "What is ROS 2?",
            "Explain nodes and topics in ROS 2",
            "What is URDF?",
            "How does Gazebo work?",
            "What is NVIDIA Isaac Sim used for?"
        ]

        print("\nRunning test queries...")
        for i, query in enumerate(test_queries, 1):
            print(f"\n{i}. Query: {query}")
            print("-" * 50)

            try:
                response = agent.ask(query)
                print(f"Response: {response}")
            except Exception as e:
                print(f"Error processing query '{query}': {e}")
                continue

            print("-" * 50)

        print("\nTesting completed!")

    except Exception as e:
        print(f"Error initializing agent: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_agent()