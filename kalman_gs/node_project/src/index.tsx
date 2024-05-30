import './index.css';

import React, { useEffect, useState } from 'react';
import ReactDOM from 'react-dom/client';

import Alerts from './components/alerts';
import Navbar from './components/navbar';
import PanelManager from './components/panel-manager';

import { alertsRef } from './modules/alerts-ref';
// Enable keyboard driving.
import './modules/keyboard-driving';
import { currentTheme } from './modules/themes';

function App() {
  const [_, setRerenderCount] = useState(0);

  useEffect(() => {
    const changeTheme = () => {
      setRerenderCount((count) => count + 1);
    };

    window.addEventListener('theme-change', changeTheme);
    return () => {
      window.removeEventListener('theme-change', changeTheme);
    };
  }, [setRerenderCount]);

  return (
    <div className='app' data-theme={currentTheme}>
      <Alerts ref={alertsRef} />
      <Navbar />
      <PanelManager />
    </div>
  );
}

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
