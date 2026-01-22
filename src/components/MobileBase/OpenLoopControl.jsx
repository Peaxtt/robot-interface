import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib'; 
import { 
  ArrowUp, ArrowDown, ArrowLeft, ArrowRight, RotateCw, RotateCcw,
  Rabbit, Turtle, Power, Ruler, Gauge, Scan, RefreshCw, CheckCircle2, Target
} from 'lucide-react';

const OpenLoopControl = ({ ros }) => {
  // --- UI STATE ---
  const [controlMode, setControlMode] = useState('VELOCITY'); 
  const [speedLevel, setSpeedLevel] = useState(1); 
  const [stepDist, setStepDist] = useState(0.5); 
  const [stepAngle, setStepAngle] = useState(90);
  const [activeBtn, setActiveBtn] = useState(null);
  const [isLocked, setIsLocked] = useState(true);
  const [isBusy, setIsBusy] = useState(false);
  
  // Align State
  const [alignStatus, setAlignStatus] = useState("IDLE");
  const [hasQR, setHasQR] = useState(false);
  const qrTimeout = useRef(null);
  
  // --- ROS STATE ---
  const cmdVel = useRef(null);
  const bridgeTopic = useRef(null);
  const moveTimer = useRef(null);
  const busyTimeout = useRef(null);

  useEffect(() => {
    if (!ros) return;

    // 1. Setup Topics
    // Velocity Mode: ต่อตรง cmd_vel เพื่อความไวสูงสุด
    cmdVel.current = new ROSLIB.Topic({ ros: ros, name: '/cmd_vel', messageType: 'geometry_msgs/msg/Twist' });
    
    // Distance/Align Mode: ส่งผ่าน Bridge Gateway
    bridgeTopic.current = new ROSLIB.Topic({ ros: ros, name: '/web_command_gateway', messageType: 'std_msgs/String' });

    // 2. Listeners
    // รับสถานะจาก Bridge
    const statusSub = new ROSLIB.Topic({ ros: ros, name: '/web_full_status', messageType: 'std_msgs/String' });
    statusSub.subscribe((msg) => {
        try {
            const data = JSON.parse(msg.data);
            
            // ถ้า Bridge บอกว่ากำลังทำ Action NAVIGATING หรือ DOCKING
            if (data.active_action !== 'IDLE' && data.active_action !== 'MISSION') {
                setIsBusy(true);
                setAlignStatus(data.feedback_msg);
            } else {
                setIsBusy(false);
                setAlignStatus("IDLE");
            }
        } catch(e) {}
    });

    // รับค่า Lector Raw เพื่อดูว่ามี QR ไหม (สำหรับปุ่ม Align)
    const lectorSub = new ROSLIB.Topic({ ros: ros, name: '/lector_floor_node/raw_data', messageType: 'std_msgs/String' });
    lectorSub.subscribe((msg) => {
        if (msg.data.includes("ID:")) {
            setHasQR(true);
            if(qrTimeout.current) clearTimeout(qrTimeout.current);
            qrTimeout.current = setTimeout(() => setHasQR(false), 500);
        }
    });

    return () => { 
        statusSub.unsubscribe(); 
        lectorSub.unsubscribe(); 
    };
  }, [ros]);

  const getSpeedValue = () => {
    switch(speedLevel) {
      case 1: return 0.2; 
      case 2: return 0.35; 
      case 3: return 0.5; 
      default: return 0.2;
    }
  };

  const publishVelocity = (lx, az) => {
    const safeLx = Math.max(-0.5, Math.min(0.5, lx));
    const safeAz = Math.max(-0.5, Math.min(0.5, az));
    if (cmdVel.current) {
        cmdVel.current.publish({ linear: { x: safeLx, y: 0, z: 0 }, angular: { x: 0, y: 0, z: safeAz } });
    }
  };

  const toggleSystem = () => {
    if (!isLocked) {
        if (moveTimer.current) clearInterval(moveTimer.current);
        if (busyTimeout.current) clearTimeout(busyTimeout.current);
        publishVelocity(0, 0);
        
        // ส่ง Stop ไปทั้ง 2 ทางเพื่อความชัวร์
        if (bridgeTopic.current) {
            bridgeTopic.current.publish({ data: JSON.stringify({ stop: true }) });
        }
        
        setActiveBtn(null); setIsBusy(false); setIsLocked(true);
    } else { setIsLocked(false); }
  };

  const handleStartAlign = () => {
      if (!bridgeTopic.current || isLocked || isBusy) return;

      console.log("🎯 Requesting Align via Bridge...");
      // ส่งคำสั่ง Align เข้า Bridge
      bridgeTopic.current.publish({ 
          data: JSON.stringify({ 
              type: 'ALIGN_DOCK', 
              qr_id: 0 // 0 = Auto detect from camera
          }) 
      });
  };

  // --- CONTROL LOGIC ---
  const handlePressStart = (direction) => {
    if (isLocked || isBusy) return;
    setActiveBtn(direction);

    // MODE 1: VELOCITY (Manual Jog) -> ต่อตรง cmd_vel
    if (controlMode === 'VELOCITY') {
        const speed = getSpeedValue();
        let lx = 0.0, az = 0.0;
        switch(direction) {
            case 'FORWARD':  lx = speed;  break;
            case 'BACKWARD': lx = -speed; break;
            case 'LEFT':     az = 0.4;  break; 
            case 'RIGHT':    az = -0.4; break; 
        }
        if (moveTimer.current) clearInterval(moveTimer.current);
        publishVelocity(lx, az);
        moveTimer.current = setInterval(() => publishVelocity(lx, az), 250); 
    } 
    // MODE 2: DISTANCE (Vector Move) -> ผ่าน Bridge
    else {
        if (!bridgeTopic.current || isBusy) return;
        // setIsBusy(true); // รอ Bridge ตอบกลับสถานะมาค่อย busy
        
        let x = 0.0, theta = 0.0;
        // Logic เดินหน้า/ถอยหลัง
        if (direction === 'FORWARD') x = parseFloat(stepDist);
        else if (direction === 'BACKWARD') x = -parseFloat(stepDist);
        
        // Logic หมุนซ้าย/ขวา
        else if (direction === 'LEFT') theta = parseFloat(stepAngle) * (Math.PI/180);
        else if (direction === 'RIGHT') theta = -parseFloat(stepAngle) * (Math.PI/180);
        
        // ส่ง JSON ไป Bridge
        bridgeTopic.current.publish({ data: JSON.stringify({
            type: 'NAVIGATE_VECTOR', 
            x: x, 
            y: 0.0, 
            theta: theta 
        })});
        
        // Timeout กันเหนียว (ถ้า Bridge ไม่ตอบ)
        busyTimeout.current = setTimeout(() => {
            setIsBusy(false); setActiveBtn(null);
        }, 5000);
    }
  };

  const handlePressEnd = () => {
    if (controlMode === 'VELOCITY' && activeBtn) {
      setActiveBtn(null);
      if (moveTimer.current) clearInterval(moveTimer.current);
      publishVelocity(0, 0);
    }
  };

  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.repeat) return;
      if (e.code === 'Space') { e.preventDefault(); toggleSystem(); return; }
      if (isLocked || isBusy) return;

      switch(e.code) {
        case 'KeyW': case 'ArrowUp':    handlePressStart('FORWARD'); break;
        case 'KeyS': case 'ArrowDown':  handlePressStart('BACKWARD'); break;
        case 'KeyA': case 'ArrowLeft':  handlePressStart('LEFT'); break;
        case 'KeyD': case 'ArrowRight': handlePressStart('RIGHT'); break;
      }
    };
    
    const handleKeyUp = (e) => {
        // เฉพาะ Velocity Mode ที่ต้องปล่อยปุ่มเพื่อหยุด
        if (controlMode === 'VELOCITY') {
            switch(e.code) {
                case 'KeyW': case 'ArrowUp':    if(activeBtn === 'FORWARD') handlePressEnd(); break;
                case 'KeyS': case 'ArrowDown':  if(activeBtn === 'BACKWARD') handlePressEnd(); break;
                case 'KeyA': case 'ArrowLeft':  if(activeBtn === 'LEFT') handlePressEnd(); break;
                case 'KeyD': case 'ArrowRight': if(activeBtn === 'RIGHT') handlePressEnd(); break;
            }
        }
    };
    
    window.addEventListener('keydown', handleKeyDown); 
    window.addEventListener('keyup', handleKeyUp);
    return () => { 
        window.removeEventListener('keydown', handleKeyDown); 
        window.removeEventListener('keyup', handleKeyUp); 
    };
  }, [isLocked, isBusy, activeBtn, speedLevel, controlMode, stepDist, stepAngle]); 

  return (
    <div className="bg-white p-4 lg:p-6 rounded-2xl border border-slate-200 shadow-sm h-full flex flex-col gap-4 select-none">
      
      {/* 1. TOP BAR */}
      <div className="flex gap-4 h-16">
          <div className={`flex-[2] bg-slate-50 rounded-xl p-1 border border-slate-200 flex gap-2 shadow-inner transition-all ${isLocked ? 'opacity-40 grayscale pointer-events-none' : ''}`}>
              <div className="grid grid-cols-2 gap-1 bg-slate-200/50 p-1 rounded-lg w-48 shrink-0">
                  <button onClick={() => setControlMode('VELOCITY')} className={`flex items-center justify-center gap-2 rounded-md text-[10px] font-bold transition-all ${controlMode === 'VELOCITY' ? 'bg-white text-blue-600 shadow-sm' : 'text-slate-400'}`}>
                      <Gauge size={30} /> VEL
                  </button>
                  <button onClick={() => setControlMode('DISTANCE')} className={`flex items-center justify-center gap-2 rounded-md text-[10px] font-bold transition-all ${controlMode === 'DISTANCE' ? 'bg-white text-purple-600 shadow-sm' : 'text-slate-400'}`}>
                      <Ruler size={30} /> DIST
                  </button>
              </div>

              <div className="flex-1 px-1 flex flex-col justify-center">
                  {controlMode === 'VELOCITY' ? (
                      <div className="flex items-center justify-between bg-white rounded-lg border border-slate-200 p-1 px-4 shadow-sm h-full">
                          <button onClick={()=>setSpeedLevel(1)} className={`flex items-center gap-1 transition-all ${speedLevel===1 ? 'text-green-500 scale-110 font-bold' : 'text-slate-300'}`}>
                              <Turtle size={26}/> <span className="text-[14px]">0.2</span>
                          </button>
                          <button onClick={()=>setSpeedLevel(2)} className={`flex items-center gap-1 transition-all ${speedLevel===2 ? 'text-blue-600 scale-110 font-bold' : 'text-slate-300'}`}>
                              <span className="text-xs">NORM</span> <span className="text-[14px]">0.35</span>
                          </button>
                          <button onClick={()=>setSpeedLevel(3)} className={`flex items-center gap-1 transition-all ${speedLevel===3 ? 'text-orange-500 scale-110 font-bold' : 'text-slate-300'}`}>
                              <Rabbit size={26}/> <span className="text-[14px]">0.5</span>
                          </button>
                      </div>
                  ) : (
                    <div className="flex gap-3 w-full h-full items-center px-2">
                      <div className="flex-1 flex items-center bg-white border border-slate-200 rounded-xl px-4 shadow-sm h-full focus-within:border-blue-400 focus-within:ring-2 focus-within:ring-blue-50">
                        <input type="number" step="0.1" value={stepDist} onChange={(e) => setStepDist(e.target.value)} className="w-full text-center text-xl font-bold text-slate-700 outline-none" />
                        <span className="text-xs font-bold text-slate-400 ml-2">M</span>
                      </div>
                      <div className="flex-1 flex items-center bg-white border border-slate-200 rounded-xl px-4 shadow-sm h-full focus-within:border-blue-400 focus-within:ring-2 focus-within:ring-blue-50">
                        <input type="number" step="15" value={stepAngle} onChange={(e) => setStepAngle(e.target.value)} className="w-full text-center text-xl font-bold text-slate-700 outline-none" />
                        <span className="text-xs font-bold text-slate-400 ml-2">DEG</span>
                      </div>
                    </div>
                  )}
                </div>
              </div>

          <button 
             onClick={handleStartAlign}
             disabled={isLocked || isBusy || !hasQR}
             className={`w-32 rounded-xl border-2 flex flex-col items-center justify-center transition-all shadow-sm
             ${alignStatus.includes('DOCKING') ? 'bg-orange-50 border-orange-400 text-orange-600'
             : hasQR && !isLocked ? 'bg-blue-600 border-blue-700 text-white animate-pulse' 
             : 'bg-slate-100 border-slate-200 text-slate-300'}`}>
             
             {alignStatus.includes('DOCKING') ? <RefreshCw size={20} className="animate-spin"/> 
              : alignStatus==='ALIGNED' ? <CheckCircle2 size={20}/>
              : <Target size={20}/>}
             <span className="text-[9px] font-bold mt-1 uppercase tracking-wider">
                 {alignStatus.includes('DOCKING') ? 'ALIGNING...' 
                  : 'ALIGN TO QR'}
             </span>
          </button>
      </div>

      {/* 2. STATUS BAR */}
      <div className={`mt-auto rounded-xl p-4 border-l-4 transition-all flex items-center gap-3 ${isLocked ? 'bg-red-50 border-red-500 text-red-600' : isBusy ? 'bg-purple-50 border-purple-500 text-purple-700' : 'bg-blue-50 border-blue-400 text-blue-600'}`}>
         <div className={`w-3 h-3 rounded-full ${isLocked ? 'bg-red-500 animate-pulse' : isBusy ? 'bg-purple-500 animate-spin' : 'bg-blue-500 animate-pulse'}`} />
         <div className="flex flex-col">
           <span className="text-[10px] font-bold uppercase">System Status</span>
           <span className="text-sm font-bold tracking-tight">
              {isLocked ? 'ENGINE LOCKED' : isBusy ? `EXECUTING: ${alignStatus}` : 'READY TO JOG'}
           </span>
         </div>
      </div>

      {/* 3. JOYSTICK */}
      <div className="flex-1 flex items-center justify-center bg-slate-50 rounded-3xl border border-slate-200 relative overflow-hidden p-6">
        <div className="relative w-full h-auto max-w-[400px] aspect-square bg-white rounded-full shadow-[0_10px_40px_-10px_rgba(0,0,0,0.1)] border border-slate-100 flex items-center justify-center transition-all duration-500">
          <div className={`absolute inset-0 rounded-full border-[3px] transition-all duration-700 ${isLocked ? 'border-transparent' : isBusy ? 'border-purple-400/30 shadow-[0_0_60px_rgba(168,85,247,0.3)]' : 'border-blue-400/30 shadow-[0_0_60px_rgba(59,130,246,0.3)]'}`} />
          
          <div className={`absolute inset-0 transition-all duration-300 ${isLocked || isBusy ? 'opacity-30 pointer-events-none' : 'opacity-100'}`}>
            <JogButton icon={<ArrowUp className="w-1/2 h-1/2" />} isActive={activeBtn === 'FORWARD'} onDown={() => handlePressStart('FORWARD')} onUp={handlePressEnd} position="top-[6%] left-1/2 -translate-x-1/2" label="FWD" color={controlMode === 'DISTANCE' ? 'purple' : 'blue'} />
            <JogButton icon={<ArrowDown className="w-1/2 h-1/2" />} isActive={activeBtn === 'BACKWARD'} onDown={() => handlePressStart('BACKWARD')} onUp={handlePressEnd} position="bottom-[6%] left-1/2 -translate-x-1/2" label="REV" color={controlMode === 'DISTANCE' ? 'purple' : 'blue'} />
            <JogButton icon={controlMode === 'VELOCITY' ? <RotateCcw className="w-1/2 h-1/2"/> : <ArrowLeft className="w-1/2 h-1/2"/>} isActive={activeBtn === 'LEFT'} onDown={() => handlePressStart('LEFT')} onUp={handlePressEnd} position="left-[6%] top-1/2 -translate-y-1/2" label={controlMode==='VELOCITY'?"ROT L":"LEFT"} color={controlMode === 'DISTANCE' ? 'purple' : 'blue'} />
            <JogButton icon={controlMode === 'VELOCITY' ? <RotateCw className="w-1/2 h-1/2"/> : <ArrowRight className="w-1/2 h-1/2"/>} isActive={activeBtn === 'RIGHT'} onDown={() => handlePressStart('RIGHT')} onUp={handlePressEnd} position="right-[6%] top-1/2 -translate-y-1/2" label={controlMode==='VELOCITY'?"ROT R":"RIGHT"} color={controlMode === 'DISTANCE' ? 'purple' : 'blue'} />
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
        <button 
            onMouseDown={onDown} onMouseUp={onUp} onMouseLeave={onUp}
            onTouchStart={(e) => { e.preventDefault(); onDown(); }} onTouchEnd={onUp}
            className={`absolute w-[22%] h-[22%] rounded-2xl flex flex-col items-center justify-center transition-all duration-100 ${position} ${isActive ? `${activeClass} shadow-lg scale-95 z-20` : 'bg-white text-slate-600 hover:bg-slate-50 border border-slate-100 shadow-sm z-10'}`}
        >
            {icon}
            <span className={`text-[10px] lg:text-xs font-bold mt-0.5 hidden sm:block ${isActive ? activeText : 'text-slate-300'}`}>{label}</span>
        </button>
    );
}

export default OpenLoopControl;