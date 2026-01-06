import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import HeroSection from '@site/src/components/HeroSection/HeroSection';
import Chatbot from '@site/src/components/Chatbot/Chatbot';

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="The Complete AI-Native Textbook for the Future of Robotics">
      <main>
        <HeroSection />
        <div style={{ padding: '40px 20px', maxWidth: '1200px', margin: '0 auto' }}>
          <h2 style={{ textAlign: 'center', marginBottom: '20px' }}>🤖 Interactive AI Assistant</h2>
          <p style={{ textAlign: 'center', marginBottom: '30px', color: '#666' }}>
            Ask questions about Physical AI & Humanoid Robotics concepts
          </p>
          <Chatbot />
        </div>
      </main>
    </Layout>
  );
}