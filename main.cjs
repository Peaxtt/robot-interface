const { app, BrowserWindow } = require('electron');

function createWindow() {
  const win = new BrowserWindow({
    width: 1160,
    height: 1280,
    webPreferences: { 
      nodeIntegration: true, 
      contextIsolation: false,
      // ✅✅✅ เพิ่มบรรทัดนี้ครับ เพื่อปลดล็อกให้ต่อ ROS Bridge ได้ ✅✅✅
      webSecurity: false, 
    },
  });
  win.loadURL('http://localhost:5173');
}

app.whenReady().then(createWindow);