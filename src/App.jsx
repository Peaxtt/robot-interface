import React, { useState, useEffect, useRef } from 'react';
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

  // --- Shared State (Memory Bank) ---
  const [sharedPath, setSharedPath] = useState([]); 
  const [sharedStep, setSharedStep] = useState(0);
  const [sharedLift, setSharedLift] = useState(0.0);
  const [sharedAngle, setSharedAngle] = useState(90);

  // Refs for Connection Logic
  const rosInstance = useRef(null);
  const reconnectTimer = useRef(null);
  const watchdogTimer = useRef(null); 
  
  const ROBOT_IP = '10.61.6.65'; 
  const ROS_PORT = '9090';

  // --- 1. ROBUST CONNECTION LOGIC ---
  useEffect(() => {
    const ROS_URL = `ws://${ROBOT_IP}:${ROS_PORT}`; 

    const connectROS = () => {
      // Prevent double connection
      if (rosInstance.current && (rosInstance.current.isConnected || rosInstance.current.socket?.readyState === 0)) {
        return;
      }

      console.log(`🔄 App: Connecting to ROS at ${ROS_URL}`);
      setRosStatus('CONNECTING');

      if (rosInstance.current) {
        try { rosInstance.current.close(); } catch(e) {}
        rosInstance.current = null;
      }

      try {
        const RosClass = ROSLIB.Ros || ROSLIB.default?.Ros;
        if (!RosClass) return;

        const newRos = new RosClass({ url: ROS_URL });
        
        newRos.on('connection', () => {
          console.log('✅ App: ROS Connected');
          setRosStatus('CONNECTED');
          setRos(newRos); 
          rosInstance.current = newRos;
        });

        newRos.on('error', (error) => {
          console.warn('⚠️ App: ROS Error'); // Less noise
          setRosStatus('ERROR');
          setRos(null);
        });

        newRos.on('close', () => {
          console.log('🔌 App: ROS Connection closed');
          setRosStatus('DISCONNECTED');
          setRos(null);
          rosInstance.current = null; 
        });

      } catch (err) {
        console.error("ROS Init Failed:", err);
        setRosStatus('ERROR');
      }
    };

    // Initial Connect
    connectROS();

    // ✅ Watchdog: Check every 3s
    watchdogTimer.current = setInterval(() => {
      const isConnected = rosInstance.current && rosInstance.current.isConnected;
      if (!isConnected) {
        console.log('🐶 Watchdog: Connection lost. Retrying...');
        connectROS();
      }
    }, 3000);

    // ✅ Visibility Listener: Reconnect immediately when app comes to foreground
    const handleVisibilityChange = () => {
      if (document.visibilityState === 'visible') {
        const isConnected = rosInstance.current && rosInstance.current.isConnected;
        if (!isConnected) {
          console.log('👀 App visible -> Force Reconnect');
          connectROS();
        }
      }
    };
    document.addEventListener('visibilitychange', handleVisibilityChange);

    return () => {
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current);
      if (watchdogTimer.current) clearInterval(watchdogTimer.current);
      document.removeEventListener('visibilitychange', handleVisibilityChange);
      if (rosInstance.current) {
        rosInstance.current.close();
        rosInstance.current = null;
      }
    };
  }, []); 

  // --- 2. SYSTEM RESET HANDLER ---
  const handleSystemReset = () => {
    if (!ros) return;
    
    // 1. Send Reset Command to Bridge
    const cmdTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/web_command_gateway',
      messageType: 'std_msgs/String'
    });
    
    const payload = JSON.stringify({ type: 'SYSTEM_RESET' });
    cmdTopic.publish({ data: payload });
    
    // 2. Force UI Reload after delay (Give time for backend to kill nodes)
    // Backend will kill 'ros2 launch' processes.
    // Ideally, we wait for user to re-launch or if using supervisor, it restarts.
    // For now, we reload page to clear frontend state.
    setTimeout(() => {
      alert("System Reset Command Sent. Reloading Interface...");
      window.location.reload();
    }, 2000);
  };

  // UI Helpers
  const getStatusBadgeColor = () => {
    switch (rosStatus) {
      case 'CONNECTED': return 'bg-green-100 text-green-700 border-green-200';
      case 'CONNECTING': return 'bg-yellow-100 text-yellow-700 border-yellow-200';
      case 'ERROR': return 'bg-red-100 text-red-700 border-red-200';
      default: return 'bg-slate-100 text-slate-500 border-slate-200';
    }
  };

  return (
    <div className="flex h-screen bg-slate-100 font-sans text-slate-900 selection:bg-blue-100">
      {/* ✅ Pass Reset Handler to Sidebar */}
      <Sidebar activeTab={activeTab} setActiveTab={setActiveTab} onSystemReset={handleSystemReset} />
      
      <main className="flex-1 flex flex-col h-screen overflow-hidden relative">
        <header className="h-16 bg-white border-b border-slate-200 flex justify-between items-center px-6 shrink-0 z-10 shadow-sm">
          <div className="flex items-center gap-4">
            <h1 className="text-lg font-bold text-slate-800 tracking-tight">
              {activeTab === 'mobile' ? 'Mobile Base Control' : 'Piggyback Module'}
            </h1>
            <span className={`border text-[10px] px-2 py-0.5 rounded font-bold flex items-center gap-1 ${getStatusBadgeColor()}`}>
              <span className={`w-1.5 h-1.5 rounded-full ${rosStatus === 'CONNECTED' ? 'bg-green-500 animate-pulse' : 'bg-current'}`}></span>
              {rosStatus}
            </span>
          </div>

          {activeTab === 'mobile' && (
            <div className="flex bg-slate-100 p-1 rounded-lg border border-slate-200">
              <button onClick={() => setControlMode('MANUAL')} className={`px-6 py-1.5 rounded-md text-xs font-bold transition-all ${controlMode === 'MANUAL' ? 'bg-white text-slate-800 shadow-sm ring-1 ring-black/5' : 'text-slate-400 hover:text-slate-600'}`}>
                  MANUAL
              </button>
              <button onClick={() => setControlMode('AUTO')} className={`px-6 py-1.5 rounded-md text-xs font-bold transition-all ${controlMode === 'AUTO' ? 'bg-white text-slate-800 shadow-sm ring-1 ring-black/5' : 'text-slate-400 hover:text-slate-600'}`}>
                  AUTO MISSION
              </button>
            </div>
          )}

          <div className="flex gap-6 font-mono text-xs">
            <div className="flex items-center gap-2">
              <span className="text-slate-500 font-bold">IP: {ROBOT_IP}</span>
            </div>
          </div>
        </header>

        <div className="flex-1 p-6 overflow-hidden bg-slate-50/50 relative">
          
          {/* TAB 1: MOBILE BASE */}
          <div className={`${activeTab === 'mobile' ? 'flex' : 'hidden'} flex-col gap-6 h-full`}>
              <div className="flex-[2] overflow-hidden relative">
                <div className={`h-full w-full ${controlMode === 'MANUAL' ? 'block' : 'hidden'}`}>
                    <OpenLoopControl ros={ros} />
                </div>
                <div className={`h-full w-full ${controlMode === 'AUTO' ? 'block' : 'hidden'}`}>
                    <ClosedLoopControl 
                        ros={ros} 
                        savedPath={sharedPath} setSavedPath={setSharedPath}
                        savedStep={sharedStep} setSavedStep={setSharedStep}
                    />
                </div>
              </div>
              <div className="h-24 shrink-0">
                <RobotStatus ros={ros} /> 
              </div>
          </div>

          {/* TAB 2: PIGGYBACK */}
          <div className={`${activeTab === 'piggyback' ? 'block' : 'hidden'} h-full`}>
             <PiggybackControl 
                ros={ros}
                savedLift={sharedLift} setSavedLift={setSharedLift}
                savedAngle={sharedAngle} setSavedAngle={setSharedAngle}
             />
          </div>

        </div>
      </main>
    </div>
  );
}

export default App;