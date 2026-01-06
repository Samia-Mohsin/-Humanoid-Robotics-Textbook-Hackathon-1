import React from 'react';
import Layout from '@theme/Layout';
import Chatbot from '@site/src/components/Chatbot/Chatbot';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function ChatbotPage() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`AI Assistant - ${siteConfig.title}`}
      description="Interactive AI assistant for Physical AI & Humanoid Robotics">
      <main style={{ padding: '20px 0', maxWidth: '1200px', margin: '0 auto' }}>
        <div style={{ padding: '0 20px' }}>
          <h1 style={{ textAlign: 'center', marginBottom: '30px' }}>
            🤖 Physical AI & Humanoid Robotics Assistant
          </h1>
          <p style={{ textAlign: 'center', marginBottom: '30px', color: '#666' }}>
            Ask questions about the textbook content, concepts, or applications
          </p>

          <Chatbot />
        </div>
      </main>
    </Layout>
  );
}