import React, { useState, useEffect } from 'react';
import * as ROSLIB from 'roslib';
import Sidebar from './components/Layout/Sidebar';
import OpenLoopControl from './components/MobileBase/OpenLoopControl';
import ClosedLoopControl from './components/MobileBase/ClosedLoopControl';
import RobotStatus from './components/MobileBase/RobotStatus'; 
import PiggybackControl from './components/Piggyback/PiggybackControl';

function App() {
  const [activeTab, setActiveTab] = useState('mobile');
  const [controlMode, setControlMode] = useState('MANUAL');
  
  const [ros, setRos] = useState(null);
  const [rosStatus, setRosStatus] = useState('DISCONNECTED'); 

  useEffect(() => {
    // ⚠️ แก้ IP ให้ตรงกับหุ่นจริง (เช่น 10.61.6.65)
    const ROS_URL = 'ws://10.61.6.65:9090';

    const connectROS = () => {
      try {
        const RosClass = ROSLIB.Ros || ROSLIB.default?.Ros;
        if (!RosClass) return;

        setRosStatus('CONNECTING');
        const newRos = new RosClass({ url: ROS_URL });

        newRos.on('connection', () => {
          console.log('✅ App: ROS Connected');
          setRosStatus('CONNECTED');
          setRos(newRos);
        });

        newRos.on('error', (error) => {
          console.warn('⚠️ App: ROS Error');
          setRosStatus('ERROR');
        });

        newRos.on('close', () => {
          console.log('🔌 App: ROS Closed');
          setRosStatus('DISCONNECTED');
          setRos(null);
          
          // Auto Reconnect ใน 3 วิ
          setTimeout(() => connectROS(), 3000); 
        });

      } catch (err) {
        console.error("App ROS Init Failed:", err);
        setRosStatus('ERROR');
      }
    };

    connectROS();

    // Cleanup เมื่อปิดเว็บ
    return () => {
      if (ros) ros.close();
    };
  }, []); // Run ครั้งเดียวตอนเปิด

  const getStatusBadgeColor = () => {
    switch (rosStatus) {
      case 'CONNECTED': return 'bg-green-100 text-green-700 border-green-200';
      case 'CONNECTING': return 'bg-yellow-100 text-yellow-700 border-yellow-200';
      case 'ERROR': return 'bg-red-100 text-red-700 border-red-200';
      default: return 'bg-slate-100 text-slate-500 border-slate-200';
    }
  };

  const getStatusText = () => {
    switch (rosStatus) {
      case 'CONNECTED': return 'ONLINE';
      case 'CONNECTING': return 'CONNECTING...';
      case 'ERROR': return 'CONNECTION ERROR';
      default: return 'OFFLINE';
    }
  };

  return (
    <div className="flex h-screen bg-slate-100 font-sans text-slate-900 selection:bg-blue-100">
      <Sidebar activeTab={activeTab} setActiveTab={setActiveTab} />

      <main className="flex-1 flex flex-col h-screen overflow-hidden relative">
        {/* Header */}
        <header className="h-16 bg-white border-b border-slate-200 flex justify-between items-center px-6 shrink-0 z-10 shadow-sm">
          <div className="flex items-center gap-4">
            <h1 className="text-lg font-bold text-slate-800 tracking-tight flex items-center gap-3">
              {activeTab === 'mobile' ? 'Mobile Base' : 'Piggyback Module'}
            </h1>
            <span className={`border text-[10px] px-2 py-0.5 rounded font-bold transition-colors duration-300 flex items-center gap-1 ${getStatusBadgeColor()}`}>
              <span className={`w-1.5 h-1.5 rounded-full ${rosStatus === 'CONNECTED' ? 'bg-green-500 animate-pulse' : 'bg-current'}`}></span>
              {getStatusText()}
            </span>
          </div>

          {activeTab === 'mobile' && (
            <div className="flex bg-slate-100 p-1 rounded-lg border border-slate-200">
              {['MANUAL', 'AUTO'].map((mode) => (
                <button
                  key={mode}
                  onClick={() => setControlMode(mode)}
                  className={`px-6 py-1.5 rounded-md text-xs font-bold transition-all shadow-sm ${
                    controlMode === mode 
                    ? 'bg-white text-slate-800 ring-1 ring-black/5' 
                    : 'text-slate-400 hover:text-slate-600 shadow-none'
                  }`}
                >
                  {mode}
                </button>
              ))}
            </div>
          )}

          <div className="flex gap-6 font-mono text-xs">
            <div className="flex items-center gap-2">
              <span className={`w-2 h-2 rounded-full ${rosStatus === 'CONNECTED' ? 'bg-green-500' : 'bg-slate-300'}`}></span>
              <span className="text-slate-500">BATTERY</span>
              <span className="text-slate-800 font-bold">{rosStatus === 'CONNECTED' ? '85%' : '--%'}</span>
            </div>
          </div>
        </header>

        {/* Content */}
        <div className="flex-1 p-6 overflow-hidden bg-slate-50/50">
          {activeTab === 'mobile' ? (
            <div className="flex flex-col gap-6 h-full">
              <div className="flex-[2] overflow-hidden">
                {/* ส่งตัวแปร ros (state) ลงไป */}
                {controlMode === 'MANUAL' 
                  ? <OpenLoopControl ros={ros} /> 
                  : <ClosedLoopControl ros={ros} />
                }
              </div>
              <div className="h-24 shrink-0">
                <RobotStatus ros={ros} /> 
              </div>
            </div>
          ) : (
             <div className="h-full">
               <PiggybackControl />
             </div>
          )}
        </div>
      </main>
    </div>
  );
}

export default App;
