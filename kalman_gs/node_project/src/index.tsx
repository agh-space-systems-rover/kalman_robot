import './index.css';

import { alertsRef } from './modules/alerts-ref';
import React from 'react';
import ReactDOM from 'react-dom/client';

import Alerts from './components/alerts';
import Navbar from './components/navbar';
import PanelManager from './components/panel-manager';

// Enable keyboard driving.
import './modules/keyboard-driving';

function App() {
  return (
    <div className='app'>
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
