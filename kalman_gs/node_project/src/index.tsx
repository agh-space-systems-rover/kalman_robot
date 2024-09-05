import './css/index.css';

import './common/gamepad-arming';
import './common/gamepad-driving';
import './common/gamepad-drilling';
import './common/science';
import './common/keyboard-driving';
import { splashRef, alertsRef, settingsRef } from './common/refs';
import { currentTheme } from './common/themes';
import React, { useEffect, useState } from 'react';
import ReactDOM from 'react-dom/client';

import Alerts from './components/alerts';
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
    </div>
  );
}

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
