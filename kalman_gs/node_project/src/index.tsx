import './css/index.css';

import { splashRef, alertsRef, settingsRef, modalRef } from './common/refs';
import { currentTheme } from './common/themes';
import React, { useEffect, useState } from 'react';
import ReactDOM from 'react-dom/client';

import Alerts from './components/alerts';
import Modal from './components/modal';
import Navbar from './components/navbar';
import PanelManager from './components/panel-manager';
import Settings from './components/settings';
import Splash from './components/splash';

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
      <Navbar />
      <PanelManager />
      <Splash ref={splashRef} />
      <Alerts ref={alertsRef} />
      <Settings ref={settingsRef} />
      <Modal ref={modalRef} />
    </div>
  );
}

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
