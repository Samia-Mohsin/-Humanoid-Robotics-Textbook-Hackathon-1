import React from 'react';
import FloatingChatbot from '@site/src/components/FloatingChatbot/FloatingChatbot';

// This is the root component that wraps the entire app
const Root = ({children}) => {
  return (
    <>
      {children}
      <FloatingChatbot />
    </>
  );
};

export default Root;