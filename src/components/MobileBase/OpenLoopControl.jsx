import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib'; 
import { 
  ArrowUp, ArrowDown, ArrowLeft, ArrowRight, 
  Rabbit, Turtle, Power, Ruler, Gauge 
} from 'lucide-react';

const OpenLoopControl = () => {
  // --- UI STATE ---
  const [controlMode, setControlMode] = useState('VELOCITY'); 
  const [speedLevel, setSpeedLevel] = useState(1);
  const [targetDistance, setTargetDistance] = useState(0.5); 
  const [activeBtn, setActiveBtn] = useState(null);
  const [isLocked, setIsLocked] = useState(true);
  const [isBusy, setIsBusy] = useState(false); 
  
  // --- ROS STATE ---
  const [isConnected, setIsConnected] = useState(false);
  const ros = useRef(null);
  const cmdVel = useRef(null);
  const moveTimer = useRef(null);
  const reconnectTimer = useRef(null);
  const stepTimeout = useRef(null); 

  // --- 1. SETUP ROS ---
  useEffect(() => {
    const ROS_URL = 'ws://127.0.0.1:9090'; // ⚠️ อย่าลืมแก้ IP เป็นหุ่นจริง

    const connectToROS = () => {
      try {
        const RosClass = ROSLIB.Ros || ROSLIB.default?.Ros;
        const TopicClass = ROSLIB.Topic || ROSLIB.default?.Topic;

        if (!RosClass || !TopicClass) return;
        if (ros.current) ros.current.close();

        ros.current = new RosClass({ url: ROS_URL });

        ros.current.on('connection', () => {
          console.log('✅ Connected');
          setIsConnected(true);
          clearTimeout(reconnectTimer.current);

          cmdVel.current = new TopicClass({
            ros: ros.current,
            name: '/turtle1/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
          });
          cmdVel.current.advertise();
        });

        ros.current.on('error', () => {
          setIsConnected(false);
          clearTimeout(reconnectTimer.current);
          reconnectTimer.current = setTimeout(connectToROS, 3000);
        });

        ros.current.on('close', () => {
          setIsConnected(false);
          clearTimeout(reconnectTimer.current);
          reconnectTimer.current = setTimeout(connectToROS, 3000);
        });

      } catch (err) {
        console.error("Init Failed:", err);
        reconnectTimer.current = setTimeout(connectToROS, 3000);
      }
    };

    connectToROS();
    return () => {
      if (ros.current) ros.current.close();
      clearTimeout(reconnectTimer.current);
    };
  }, []);

  // --- 2. LOGIC ---
  const getSpeedValue = () => {
    switch(speedLevel) {
      case 1: return 0.1;
      case 2: return 0.3;
      case 3: return 0.5;
      default: return 0.1;
    }
  };

  const publishVelocity = (lx, az) => {
    lx = Math.max(-0.5, Math.min(0.5, lx));
    az = Math.max(-0.5, Math.min(0.5, az));

    if (isConnected && cmdVel.current) {
        const twist = {
            linear: { x: lx, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: az }
        };
        cmdVel.current.publish(twist);
    }
  };

  const toggleSystem = () => {
    if (!isLocked) {
        console.log("🚨 EMERGENCY STOP");
        if (moveTimer.current) clearInterval(moveTimer.current);
        if (stepTimeout.current) clearTimeout(stepTimeout.current);
        publishVelocity(0.0, 0.0);
        setActiveBtn(null);
        setIsBusy(false);
        setIsLocked(true);
    } else {
        setIsLocked(false);
    }
  };

  // --- HANDLE MOVEMENT ---
  const handlePressStart = (direction) => {
    if (isLocked || isBusy) return;
    setActiveBtn(direction);

    // ✅✅✅ แก้ตรงนี้: ถ้าโหมด Distance ให้ใช้ความเร็ว 0.5 เสมอ
    const speed = controlMode === 'DISTANCE' ? 0.5 : getSpeedValue();
    
    let lx = 0.0, az = 0.0;

    switch(direction) {
        case 'FORWARD':  lx = speed;  break;
        case 'BACKWARD': lx = -speed; break;
        case 'LEFT':     az = speed;  break;
        case 'RIGHT':    az = -speed; break;
    }

    if (controlMode === 'VELOCITY') {
        // Continuous Mode
        if (moveTimer.current) clearInterval(moveTimer.current);
        publishVelocity(lx, az);
        moveTimer.current = setInterval(() => publishVelocity(lx, az), 250);

    } else {
        // Distance Mode (Fixed Speed 0.5)
        setIsBusy(true);
        
        let duration = (targetDistance / Math.abs(speed)) * 1000;
        
        console.log(`Step: ${targetDistance}m @ ${speed}m/s (${duration.toFixed(0)}ms)`);

        publishVelocity(lx, az);
        const interval = setInterval(() => publishVelocity(lx, az), 250);
        moveTimer.current = interval; 

        stepTimeout.current = setTimeout(() => {
            clearInterval(interval);
            publishVelocity(0.0, 0.0);
            setActiveBtn(null);
            setIsBusy(false);
            console.log("Step Finished");
        }, duration);
    }
  };

  const handlePressEnd = () => {
    if (controlMode === 'VELOCITY' && activeBtn) {
      setActiveBtn(null);
      if (moveTimer.current) clearInterval(moveTimer.current);
      publishVelocity(0.0, 0.0);
    }
  };

  // --- 3. KEYBOARD ---
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.repeat) return;
      if (e.code === 'Space') { e.preventDefault(); toggleSystem(); return; }
      if (isLocked) return; 

      switch(e.key.toLowerCase()) {
        case 'w': case 'arrowup':    handlePressStart('FORWARD'); break;
        case 's': case 'arrowdown':  handlePressStart('BACKWARD'); break;
        case 'a': case 'arrowleft':  handlePressStart('LEFT'); break;
        case 'd': case 'arrowright': handlePressStart('RIGHT'); break;
      }
    };

    const handleKeyUp = (e) => {
        if (controlMode === 'VELOCITY') {
            switch(e.key.toLowerCase()) {
                case 'w': case 'arrowup':    if(activeBtn === 'FORWARD') handlePressEnd(); break;
                case 's': case 'arrowdown':  if(activeBtn === 'BACKWARD') handlePressEnd(); break;
                case 'a': case 'arrowleft':  if(activeBtn === 'LEFT') handlePressEnd(); break;
                case 'd': case 'arrowright': if(activeBtn === 'RIGHT') handlePressEnd(); break;
            }
        }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [isLocked, activeBtn, speedLevel, controlMode, targetDistance, isBusy]);

  // --- RENDER UI ---
  return (
    <div className="bg-white p-4 lg:p-6 rounded-2xl border border-slate-200 shadow-sm h-full flex flex-col lg:flex-row gap-6 select-none overflow-hidden relative">
      <div className="lg:w-1/3 flex flex-col gap-4 lg:gap-6 shrink-0 mt-8 lg:mt-0"> 
        <div className={`bg-slate-50 rounded-2xl p-1.5 border border-slate-200 flex flex-col gap-3 shadow-inner transition-all duration-500 ${isLocked ? 'opacity-40 grayscale pointer-events-none' : ''}`}>
            <div className="grid grid-cols-2 gap-1 bg-slate-200/50 p-1 rounded-xl">
                <button onClick={() => setControlMode('VELOCITY')} className={`flex items-center justify-center gap-2 py-2 rounded-lg text-xs font-bold transition-all ${controlMode === 'VELOCITY' ? 'bg-white text-blue-600 shadow-sm' : 'text-slate-400 hover:text-slate-600'}`}>
                    <Gauge size={16} /> Velocity
                </button>
                <button onClick={() => setControlMode('DISTANCE')} className={`flex items-center justify-center gap-2 py-2 rounded-lg text-xs font-bold transition-all ${controlMode === 'DISTANCE' ? 'bg-white text-purple-600 shadow-sm' : 'text-slate-400 hover:text-slate-600'}`}>
                    <Ruler size={16} /> Distance
                </button>
            </div>

            <div className="p-2 flex flex-col gap-3">
                {controlMode === 'VELOCITY' ? (
                    <>
                        <div className="flex items-center justify-between bg-white rounded-xl border border-slate-200 p-1 shadow-sm h-12 relative overflow-hidden">
                            <div className={`absolute top-1 bottom-1 w-1/3 bg-blue-100 rounded-lg transition-all duration-300 ${speedLevel === 1 ? 'left-1' : speedLevel === 2 ? 'left-[33%]' : 'left-[66%]'}`} />
                            <button onClick={() => setSpeedLevel(1)} className={`flex-1 z-10 flex justify-center items-center transition-all ${speedLevel === 1 ? 'text-blue-600 scale-110' : 'text-slate-400'}`}><Turtle size={20}/></button>
                            <button onClick={() => setSpeedLevel(2)} className={`flex-1 z-10 flex justify-center items-center font-bold text-xs transition-all ${speedLevel === 2 ? 'text-blue-600' : 'text-slate-300'}`}>NORM</button>
                            <button onClick={() => setSpeedLevel(3)} className={`flex-1 z-10 flex justify-center items-center transition-all ${speedLevel === 3 ? 'text-orange-500 scale-110' : 'text-slate-400'}`}><Rabbit size={20}/></button>
                        </div>
                        <div className="text-center font-mono font-bold text-slate-500 text-xs">Continuous Mode: {getSpeedValue()} m/s</div>
                    </>
                ) : (
                    <>
                        <div className="grid grid-cols-3 gap-2">
                            {[0.1, 0.5, 1.0].map((dist) => (
                                <button key={dist} onClick={() => setTargetDistance(dist)} className={`py-3 rounded-xl border font-mono font-bold text-sm transition-all ${targetDistance === dist ? 'bg-purple-100 border-purple-300 text-purple-700 shadow-sm' : 'bg-white border-slate-200 text-slate-400 hover:bg-slate-50'}`}>
                                    {dist}m
                                </button>
                            ))}
                        </div>
                        <div className="text-center font-mono font-bold text-slate-500 text-xs">Step Mode: Move {targetDistance}m (Fixed 0.5m/s)</div>
                    </>
                )}
            </div>
        </div>

        <div className={`mt-auto rounded-xl p-4 border-l-4 transition-all flex items-center gap-3 ${isLocked ? 'bg-red-50 border-red-500 text-red-600' : isBusy ? 'bg-purple-50 border-purple-500 text-purple-700' : isConnected ? 'bg-green-50 border-green-500 text-green-700' : 'bg-blue-50 border-blue-400 text-blue-600'}`}>
           <div className={`w-3 h-3 rounded-full ${isLocked ? 'bg-red-500 animate-pulse' : isBusy ? 'bg-purple-500 animate-spin' : isConnected ? 'bg-green-500 animate-ping' : 'bg-blue-500 animate-pulse'}`} />
           <div className="flex flex-col">
             <span className="text-[10px] font-bold uppercase">System Status</span>
             <span className="text-sm font-bold tracking-tight">
                {isLocked ? 'ENGINE LOCKED' : isBusy ? 'MOVING STEP...' : isConnected ? 'READY TO JOG' : 'ROS DISCONNECTED'}
             </span>
           </div>
        </div>
      </div>

      <div className="flex-1 flex items-center justify-center bg-slate-50 rounded-3xl border border-slate-200 relative overflow-hidden p-6">
        <div className="absolute inset-0 opacity-[0.03] bg-[radial-gradient(#000_1px,transparent_1px)] [background-size:16px_16px]" />
        <div className="relative w-full h-auto max-w-[400px] aspect-square bg-white rounded-full shadow-[0_10px_40px_-10px_rgba(0,0,0,0.1)] border border-slate-100 flex items-center justify-center transition-all duration-500">
          <div className={`absolute inset-0 rounded-full border-[3px] transition-all duration-700 ${isLocked ? 'border-transparent' : isBusy ? 'border-purple-400/30 shadow-[0_0_60px_rgba(168,85,247,0.3)]' : 'border-blue-400/30 shadow-[0_0_60px_rgba(59,130,246,0.3)]'}`} />
          <div className={`absolute inset-0 transition-all duration-300 ${isLocked ? 'opacity-30 grayscale pointer-events-none' : 'opacity-100'}`}>
            <JogButton icon={<ArrowUp className="w-1/2 h-1/2" />} isActive={activeBtn === 'FORWARD'} onDown={() => handlePressStart('FORWARD')} onUp={handlePressEnd} position="top-[6%] left-1/2 -translate-x-1/2" label="FWD" color={controlMode === 'DISTANCE' ? 'purple' : 'blue'} />
            <JogButton icon={<ArrowDown className="w-1/2 h-1/2" />} isActive={activeBtn === 'BACKWARD'} onDown={() => handlePressStart('BACKWARD')} onUp={handlePressEnd} position="bottom-[6%] left-1/2 -translate-x-1/2" label="REV" color={controlMode === 'DISTANCE' ? 'purple' : 'blue'} />
            <JogButton icon={<ArrowLeft className="w-1/2 h-1/2" />} isActive={activeBtn === 'LEFT'} onDown={() => handlePressStart('LEFT')} onUp={handlePressEnd} position="left-[6%] top-1/2 -translate-y-1/2" label="LEFT" color={controlMode === 'DISTANCE' ? 'purple' : 'blue'} />
            <JogButton icon={<ArrowRight className="w-1/2 h-1/2" />} isActive={activeBtn === 'RIGHT'} onDown={() => handlePressStart('RIGHT')} onUp={handlePressEnd} position="right-[6%] top-1/2 -translate-y-1/2" label="RIGHT" color={controlMode === 'DISTANCE' ? 'purple' : 'blue'} />
          </div>
          <button onClick={toggleSystem} className={`w-[20%] h-[20%] rounded-full flex items-center justify-center relative z-20 transition-all duration-300 group ${isLocked ? 'bg-white text-red-500 border-2 border-red-100 shadow-[0_0_15px_rgba(239,68,68,0.3)] animate-pulse' : 'bg-white text-blue-500 shadow-[0_0_20px_rgba(59,130,246,0.5)] scale-110 ring-4 ring-blue-50 border-0'}`}>
             <Power size={24} strokeWidth={3} className={isLocked ? '' : 'drop-shadow-md'} />
          </button>
          {isLocked && <div className="absolute top-[65%] left-1/2 -translate-x-1/2 text-red-400 font-bold text-[10px] tracking-widest animate-pulse pointer-events-none whitespace-nowrap bg-white/90 px-3 py-1 rounded-full shadow-sm border border-red-100 z-10">PRESS TO UNLOCK</div>}
        </div>
      </div>
    </div>
  );
};

const JogButton = ({ icon, isActive, onDown, onUp, position, label, color = 'blue' }) => {
    const activeClass = color === 'purple' ? 'bg-purple-500 text-white shadow-purple-200' : 'bg-blue-500 text-white shadow-blue-200';
    const activeText = color === 'purple' ? 'text-purple-100' : 'text-blue-100';
    return (
        <button onMouseDown={onDown} onMouseUp={onUp} onMouseLeave={onUp} onTouchStart={(e) => { e.preventDefault(); onDown(); }} onTouchEnd={onUp} className={`absolute w-[22%] h-[22%] rounded-2xl flex flex-col items-center justify-center transition-all duration-100 ${position} ${isActive ? `${activeClass} shadow-lg scale-95 z-20` : 'bg-white text-slate-600 hover:bg-slate-50 border border-slate-100 shadow-sm z-10'}`}>
            {icon}
            <span className={`text-[10px] lg:text-xs font-bold mt-0.5 hidden sm:block ${isActive ? activeText : 'text-slate-300'}`}>{label}</span>
        </button>
    );
}

export default OpenLoopControl;