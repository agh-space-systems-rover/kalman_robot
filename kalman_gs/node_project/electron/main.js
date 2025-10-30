const { app, BrowserWindow, screen } = require('electron');

const DEV_URL = 'http://localhost:3000';

app.setName('DISTOBEE Ground Station');

function createWindow() {
  const window = new BrowserWindow({
    show: false,
    fullscreen: false,
    frame: true,
    autoHideMenuBar: true,
    webPreferences: {}
  });

  window.loadURL(DEV_URL).then(response => {
    // nothing to do
  });

  window.once('ready-to-show', () => {
    window.maximize();
    const { width, height } = screen.getPrimaryDisplay().workAreaSize;
    window.setBounds({ x: 0, y: 0, width, height });
    window.show();
    window.focus();
  });

  // window.webContents.openDevTools({ mode: 'detach' });
}

app.whenReady().then(() => {
  createWindow();
  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});
