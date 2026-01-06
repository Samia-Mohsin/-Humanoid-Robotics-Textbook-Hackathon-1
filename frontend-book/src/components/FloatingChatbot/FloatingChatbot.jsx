import React, { useState } from 'react';
import styles from './FloatingChatbot.module.css';
import ChatbotComponent from '../Chatbot/Chatbot';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChatbot = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      {isOpen && (
        <div className={styles.chatbotContainer}>
          <div className={styles.chatbot}>
            <div className={styles.chatbotHeader}>
              <span>🤖 AI Assistant</span>
              <button
                className={styles.closeButton}
                onClick={toggleChatbot}
                aria-label="Close chat"
              >
                ×
              </button>
            </div>
            <div style={{ height: 'calc(100% - 60px)' }}>
              <ChatbotComponent />
            </div>
          </div>
        </div>
      )}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.hidden : ''}`}
        onClick={toggleChatbot}
        aria-label="Open chat"
      >
        🤖
      </button>
    </>
  );
};

export default FloatingChatbot;