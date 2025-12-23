import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import HeroSection from '@site/src/components/HeroSection/HeroSection';

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="The Complete AI-Native Textbook for the Future of Robotics">
      <main>
        <HeroSection />
      </main>
    </Layout>
  );
}