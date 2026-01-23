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

  const rosInstance = useRef(null);
  const reconnectTimer = useRef(null);
  
  const ROBOT_IP = '10.61.6.65'; 
  const ROS_PORT = '9090';

  useEffect(() => {
    const ROS_URL = `ws://${ROBOT_IP}:${ROS_PORT}`; 
    console.log(`Connecting to ROS at: ${ROS_URL}`);

    const connectROS = () => {
      if (rosInstance.current) rosInstance.current.close();
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current);

      try {
        const RosClass = ROSLIB.Ros || ROSLIB.default?.Ros;
        if (!RosClass) return;

        setRosStatus('CONNECTING');
        const newRos = new RosClass({ url: ROS_URL });
        rosInstance.current = newRos; 

        newRos.on('connection', () => {
          console.log(' App: ROS Connected');
          setRosStatus('CONNECTED');
          setRos(newRos); 
        });

        newRos.on('error', (error) => {
          console.log('ROS Error:', error);
          setRosStatus('ERROR');
        });

        newRos.on('close', () => {
          setRosStatus('DISCONNECTED');
          setRos(null);
          clearTimeout(reconnectTimer.current);
          reconnectTimer.current = setTimeout(connectROS, 5000); 
        });

      } catch (err) {
        setRosStatus('ERROR');
      }
    };

    connectROS();

    return () => {
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current);
      if (rosInstance.current) rosInstance.current.close();
    };
  }, []); 

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
      <Sidebar activeTab={activeTab} setActiveTab={setActiveTab} />
      
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
          
          {/* --- TAB 1: MOBILE BASE --- */}
          <div className={`${activeTab === 'mobile' ? 'flex' : 'hidden'} flex-col gap-6 h-full`}>
              
              <div className="flex-[2] overflow-hidden relative">
                
                {/* ✅ FIX: ใช้ Hidden Mode แทนการ Unmount 
                   ทำให้ทั้ง Manual และ Auto ทำงานค้างไว้ตลอดเวลา ไม่รีเซ็ตค่า 
                */}

                {/* 1. MANUAL CONTROL */}
                <div className={`h-full w-full ${controlMode === 'MANUAL' ? 'block' : 'hidden'}`}>
                    <OpenLoopControl ros={ros} />
                </div>

                {/* 2. AUTO MISSION */}
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

          {/* --- TAB 2: PIGGYBACK --- */}
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