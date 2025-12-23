import React from 'react';
import Link from '@docusaurus/Link';
import styles from './HeroSection.module.css';

const HeroSection = () => {
  return (
    <section className={styles.heroSection}>
      <div className={styles.contentContainer}>
        <div className={styles.robotIcon}>
          <img
            src="/img/hero-robot.svg"
            alt="Physical AI Robot Mascot"
            className={`${styles.robotImage} hero-robot-medium`}
          />
        </div>
        <div className={styles.textContainer}>
          <h1 className={styles.title}>
            Physical AI & Humanoid Robotics
          </h1>
          <p className={styles.subtitle}>
            The Complete AI-Native Textbook
          </p>
          <Link
            className={styles.ctaButton}
            to="/docs/intro">
            Start Reading
          </Link>
        </div>
      </div>
    </section>
  );
};

export default HeroSection;