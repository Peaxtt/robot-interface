import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';
import { 
  MoveVertical, RotateCw, Settings, 
  ChevronLeft, ChevronRight, ArrowDown, 
  Lock, Unlock, RefreshCw, Target, ChevronsLeft, ChevronsRight
} from 'lucide-react';

const PiggybackControl = ({ ros, savedLift, setSavedLift, savedAngle, setSavedAngle }) => {
  // --- CONFIG ---
  const LEVEL_MAP = [0.0, 0.25, 0.55, 0.85]; // Level 1-4
  const MAX_HEIGHT = 1.0;
  const MAX_SLIDE = 0.5;
  const TIMEOUT_SEC = 20;

  // Component IDs
  const COMPONENT = { LIFT: 0, TURNTABLE: 1, INSERT: 2, HOOK: 3 };

  // --- STATE ---
  const [status, setStatus] = useState("READY");
  const [isBusy, setIsBusy] = useState(false);
  const [isError, setIsError] = useState(false);

  // Lift State
  const liftHeight = savedLift || 0.0;
  const setLiftHeight = setSavedLift || (() => {});   
  const [targetLift, setTargetLift] = useState(liftHeight);    
  const [tempLift, setTempLift] = useState(0.0);        

  // Turn State
  const turntableAngle = savedAngle || 90;
  const setTurntableAngle = setSavedAngle || (() => {}); 
  const [activeDir, setActiveDir] = useState('ROBOT'); 
  const [tempAngle, setTempAngle] = useState(90);

  // Tools State
  const [slideDist, setSlideDist] = useState(0.0);
  const [targetSlide, setTargetSlide] = useState(0.0); 
  const [tempSlide, setTempSlide] = useState(0.0);     
  const [isHookLocked, setIsHookLocked] = useState(false);

  // Refs
  const bridgeTopic = useRef(null);
  const timeoutRef = useRef(null);

  // --- ROS SETUP ---
  useEffect(() => {
    if (ros) {
      bridgeTopic.current = new ROSLIB.Topic({
        ros: ros, name: '/web_command_gateway', messageType: 'std_msgs/String'
      });

      const statusSub = new ROSLIB.Topic({
        ros: ros, name: '/web_full_status', messageType: 'std_msgs/String'
      });

      statusSub.subscribe((msg) => {
        try {
            const data = JSON.parse(msg.data);
            const action = data.active_action || "IDLE";
            const feedback = data.feedback_msg || "";

            if (action === "IDLE") {
                clearTimeoutHandler();
                setIsBusy(false);
                setIsError(false);
                setStatus("READY");
            } else {
                if (!isError) {
                    setIsBusy(true);
                    setStatus(feedback || `BUSY: ${action}`);
                }
            }
        } catch (e) { console.error(e); }
      });

      return () => { statusSub.unsubscribe(); clearTimeoutHandler(); };
    }
  }, [ros]);

  const clearTimeoutHandler = () => {
    if (timeoutRef.current) { clearTimeout(timeoutRef.current); timeoutRef.current = null; }
  };

  const startTimeout = () => {
    clearTimeoutHandler();
    timeoutRef.current = setTimeout(() => {
        setIsError(true);
        setStatus("TIMEOUT: CHECK ROBOT!");
    }, TIMEOUT_SEC * 1000);
  };

  // --- SIMULATION (Visual Only - ทำให้ UI ขยับสวยๆ) ---
  useEffect(() => {
    const interval = setInterval(() => {
      setLiftHeight(prev => {
        const diff = targetLift - prev;
        return Math.abs(diff) < 0.01 ? targetLift : prev + (diff * 0.15);
      });
      setSlideDist(prev => {
        const diff = targetSlide - prev;
        return Math.abs(diff) < 0.01 ? targetSlide : prev + (diff * 0.15);
      });
    }, 30);
    return () => clearInterval(interval);
  }, [targetLift, targetSlide, setLiftHeight]);

  // --- ACTIONS ---
  const sendActionCommand = (compId, value, desc) => {
    if (isBusy && !isError && compId !== COMPONENT.HOOK) return;

    setIsError(false);
    setIsBusy(true);
    setStatus(`EXECUTING: ${desc}...`);
    startTimeout();

    if (bridgeTopic.current) {
      const payload = { type: 'PIGGYBACK_MANUAL', component: compId, value: value };
      bridgeTopic.current.publish({ data: JSON.stringify(payload) });
    }
  };

  // --- HANDLERS ---

  const handleLiftLevel = (lvl) => { 
      setTargetLift(LEVEL_MAP[lvl]);
      sendActionCommand(COMPONENT.LIFT, lvl, `LIFT LVL ${lvl+1}`); // ส่ง Index 0,1,2,3
  };

  // ✅ LOGIC SWAP: Turn
  const handleTurn = (dir) => {
    let idx = 0;
    let visualAngle = 90;

    if (dir === 'RIGHT') { idx = 0; visualAngle = 0; }
    if (dir === 'ROBOT') { idx = 1; visualAngle = 90; } // Robot(Back) = Index 1
    if (dir === 'LEFT')  { idx = 2; visualAngle = 180;} // Left = Index 2
    
    setActiveDir(dir);
    setTurntableAngle(visualAngle);
    setTempAngle(visualAngle);
    sendActionCommand(COMPONENT.TURNTABLE, idx, `TURN ${dir}`);
  };

  // ✅ LOGIC SWAP: Hook
  const toggleHook = () => {
    const newState = !isHookLocked;
    setIsHookLocked(newState);
    // newState=True (เขียว/Lock) -> ส่ง 0
    // newState=False (เทา/Unlock) -> ส่ง 1
    sendActionCommand(COMPONENT.HOOK, newState ? 0 : 1, newState ? "LOCKING" : "UNLOCKING");
  };

  const handleSlideQuick = (val) => { 
    const idx = val === 0 ? 0 : 1;
    setTargetSlide(val);
    sendActionCommand(COMPONENT.INSERT, idx, idx === 0 ? "SLIDE IN" : "SLIDE OUT");
  };

  // Manual Handlers (Visual Only)
  const handleManualSetLift = () => setTargetLift(tempLift);
  const handleManualSetTurn = () => setTurntableAngle(tempAngle);
  const handleManualSetSlide = () => setTargetSlide(tempSlide);

  const disabledClass = isBusy ? 'opacity-50 cursor-not-allowed pointer-events-none' : '';

  return (
    <div className="h-full w-full p-2 select-none font-sans text-slate-700 bg-slate-50/50">
      
      <div className="flex gap-6 h-full max-w-[1920px] mx-auto">
        
        {/* LIFT */}
        <div className="flex-1 bg-white rounded-[2rem] p-6 shadow-xl border border-slate-100 flex flex-col relative overflow-hidden">
            <div className="flex justify-between items-center mb-6">
                <div className="flex items-center gap-3">
                    <div className="p-3 bg-blue-50 rounded-2xl text-blue-600"><MoveVertical size={24}/></div>
                    <div>
                        <h2 className="text-lg font-black text-slate-800">LIFT AXIS</h2>
                        <p className="text-[10px] font-bold text-slate-400">HEIGHT CONTROL</p>
                    </div>
                </div>
                <div className="text-right">
                    <div className="text-3xl font-black text-slate-800 tabular-nums">{liftHeight.toFixed(2)}</div>
                    <div className="text-[9px] font-bold text-slate-400">Meters</div>
                </div>
            </div>
            <div className="flex-1 flex gap-6 min-h-0">
                <div className="w-16 h-full bg-slate-50 rounded-full border border-slate-200 relative shadow-inner flex flex-col items-center overflow-hidden">
                    <div className="absolute inset-0 flex flex-col justify-between py-6 opacity-20 px-4">
                        {[...Array(20)].map((_,i) => <div key={i} className="w-full h-0.5 bg-slate-400"/>)}
                    </div>
                    <div className="absolute w-12 h-10 bg-blue-600 rounded-xl shadow-lg border-2 border-white transition-all duration-300 ease-out z-20"
                        style={{ bottom: `${(liftHeight / MAX_HEIGHT) * 92}%` }}>
                    </div>
                </div>
                <div className="flex-1 flex flex-col gap-3 h-full overflow-y-auto pr-1">
                    {[3, 2, 1, 0].map((lvl) => {
                        const isActive = Math.abs(targetLift - LEVEL_MAP[lvl]) < 0.01;
                        return (
                            <button key={lvl} onClick={() => handleLiftLevel(lvl)} disabled={isBusy}
                                className={`flex-1 min-h-[60px] w-full rounded-2xl border-2 transition-all duration-200 flex items-center justify-between px-5
                                ${isActive ? 'bg-slate-800 border-slate-800 text-white shadow-lg' : 'bg-white border-slate-100 text-slate-500 hover:border-blue-200'}
                                ${disabledClass}`}>
                                <div className="flex flex-col items-start">
                                    <span className="font-black text-sm">LEVEL {lvl + 1}</span>
                                    <span className={`text-[10px] font-mono font-bold ${isActive ? 'text-slate-400' : 'text-slate-300'}`}>{LEVEL_MAP[lvl]}m</span>
                                </div>
                                {isActive && <div className="w-2 h-2 bg-green-400 rounded-full animate-pulse"/>}
                            </button>
                        );
                    })}
                </div>
            </div>
            <div className="mt-6 pt-4 border-t border-slate-100 flex items-center justify-between">
                <span className="text-[10px] font-bold text-slate-400 uppercase">Manual Height</span>
                <div className="flex bg-slate-50 p-1 rounded-xl border border-slate-200 w-40">
                    <input type="number" step="0.01" value={tempLift} onChange={(e) => setTempLift(parseFloat(e.target.value))} disabled={isBusy} className="w-full bg-transparent px-2 text-center font-bold text-slate-700 text-sm focus:outline-none"/>
                    <button onClick={handleManualSetLift} disabled={isBusy} className="bg-white px-3 py-1.5 rounded-lg text-[10px] font-bold text-slate-500 shadow-sm border border-slate-100">GO</button>
                </div>
            </div>
        </div>

        {/* ROTATION */}
        <div className="flex-1 bg-white rounded-[2rem] p-6 shadow-xl border border-slate-100 flex flex-col relative">
            <div className="flex justify-between items-center mb-4">
                <div className="flex items-center gap-3">
                    <div className="p-3 bg-purple-50 rounded-2xl text-purple-600"><RotateCw size={24}/></div>
                    <div>
                        <h2 className="text-lg font-black text-slate-800">ROTATION</h2>
                        <p className="text-[10px] font-bold text-slate-400">TURNTABLE</p>
                    </div>
                </div>
                <div className="text-right">
                    <div className="text-3xl font-black text-slate-800 tabular-nums">{turntableAngle.toFixed(0)}°</div>
                    <div className="text-[9px] font-bold text-slate-400 uppercase">Angle</div>
                </div>
            </div>
            <div className="flex-1 relative flex items-center justify-center min-h-0">
                <button onClick={() => handleTurn('LEFT')} disabled={isBusy} className={`absolute left-2 top-1/2 -translate-y-1/2 w-16 h-16 rounded-2xl flex flex-col items-center justify-center shadow-lg transition-all border-2 z-30 ${activeDir === 'LEFT' ? 'bg-purple-600 border-purple-600 text-white scale-110' : 'bg-white border-white text-slate-400 hover:text-purple-600'} ${disabledClass}`}>
                    <ChevronLeft size={24}/><span className="text-[9px] font-bold mt-1">L</span>
                </button>
                <button onClick={() => handleTurn('RIGHT')} disabled={isBusy} className={`absolute right-2 top-1/2 -translate-y-1/2 w-16 h-16 rounded-2xl flex flex-col items-center justify-center shadow-lg transition-all border-2 z-30 ${activeDir === 'RIGHT' ? 'bg-purple-600 border-purple-600 text-white scale-110' : 'bg-white border-white text-slate-400 hover:text-purple-600'} ${disabledClass}`}>
                    <ChevronRight size={24}/><span className="text-[9px] font-bold mt-1">R</span>
                </button>
                <button onClick={() => handleTurn('ROBOT')} disabled={isBusy} className={`absolute bottom-20 left-1/2 -translate-x-1/2 w-16 h-16 rounded-2xl flex flex-col items-center justify-center shadow-lg transition-all border-2 z-30 ${activeDir === 'ROBOT' ? 'bg-purple-600 border-purple-600 text-white scale-110' : 'bg-white border-white text-slate-400 hover:text-purple-600'} ${disabledClass}`}>
                    <ArrowDown size={24}/><span className="text-[9px] font-bold mt-1">IN</span>
                </button>
                <div className="w-64 h-64 rounded-full bg-slate-50 border border-slate-100 flex items-center justify-center relative">
                    <div className="w-48 h-48 rounded-full flex items-center justify-center transition-transform duration-700 ease-out relative z-10" style={{ transform: `rotate(${turntableAngle}deg)` }}>
                        <div className="absolute right-0 w-32 h-24 bg-slate-800 rounded-r-3xl border-4 border-slate-700 shadow-xl flex items-center justify-end pr-4">
                            <div className="text-white/20"><ChevronRight size={28}/></div>
                        </div>
                        <div className="w-10 h-10 bg-purple-500 rounded-full shadow-lg border-4 border-white z-20 relative"/>
                    </div>
                </div>
            </div>
            <div className="mt-4 pt-4 border-t border-slate-100 flex items-center justify-between">
                <span className="text-[10px] font-bold text-slate-400 uppercase">Manual Angle</span>
                <div className="flex bg-slate-50 p-1 rounded-xl border border-slate-200 w-40">
                    <input type="number" value={tempAngle} onChange={(e) => setTempAngle(parseFloat(e.target.value))} disabled={isBusy} className="w-full bg-transparent px-2 text-center font-bold text-slate-700 text-sm focus:outline-none"/>
                    <button onClick={handleManualSetTurn} disabled={isBusy} className="bg-white px-3 py-1.5 rounded-lg text-[10px] font-bold text-slate-500 shadow-sm border border-slate-100">GO</button>
                </div>
            </div>
        </div>

        {/* TOOLS */}
        <div className="flex-1 flex flex-col gap-6">
            <div className="flex-[3] bg-white rounded-[2rem] p-6 shadow-xl border border-slate-100 flex flex-col">
                <div className="flex justify-between items-center mb-6">
                    <div className="flex items-center gap-3">
                        <div className="p-3 bg-orange-50 rounded-2xl text-orange-600"><Settings size={24}/></div>
                        <div>
                            <h2 className="text-lg font-black text-slate-800">SLIDE</h2>
                            <p className="text-[10px] font-bold text-slate-400">EXTENSION</p>
                        </div>
                    </div>
                    <div className="text-right">
                        <div className="text-3xl font-black text-slate-800 tabular-nums">{slideDist.toFixed(2)}</div>
                        <div className="text-[9px] font-bold text-slate-400">Meters</div>
                    </div>
                </div>
                <div className="relative h-12 bg-slate-50 rounded-2xl border border-slate-200 mb-8 overflow-hidden flex items-center px-2">
                    <div className="absolute left-0 top-0 bottom-0 bg-orange-400 opacity-20 transition-all duration-300" style={{width: `${(targetSlide/MAX_SLIDE)*100}%`}}></div>
                    <div className="absolute h-8 w-1 bg-orange-500 rounded-full transition-all duration-300 shadow-[0_0_10px_rgba(249,115,22,0.5)]" style={{left: `${(targetSlide/MAX_SLIDE)*98}%`}}></div>
                    <div className="absolute inset-0 flex justify-between px-3 items-center opacity-40 text-[9px] font-bold"><span>HOME</span><span>MAX</span></div>
                </div>
                <div className="flex gap-4 mb-4 flex-1 min-h-0">
                    <button onClick={() => handleSlideQuick(0.0)} disabled={isBusy} className={`flex-1 bg-slate-100 rounded-xl border border-slate-200 text-slate-500 font-bold text-xs hover:bg-orange-50 hover:text-orange-600 hover:border-orange-200 transition-all active:scale-95 flex flex-col items-center justify-center gap-1 ${disabledClass}`}>
                        <ChevronsLeft size={24}/> SLIDE IN
                    </button>
                    <button onClick={() => handleSlideQuick(MAX_SLIDE)} disabled={isBusy} className={`flex-1 bg-slate-100 rounded-xl border border-slate-200 text-slate-500 font-bold text-xs hover:bg-orange-50 hover:text-orange-600 hover:border-orange-200 transition-all active:scale-95 flex flex-col items-center justify-center gap-1 ${disabledClass}`}>
                        <ChevronsRight size={24}/> SLIDE OUT
                    </button>
                </div>
                <div className="mt-auto pt-4 border-t border-slate-100 flex items-center justify-between">
                    <span className="text-[10px] font-bold text-slate-400 uppercase">Manual Dist</span>
                    <div className="flex bg-slate-50 p-1 rounded-xl border border-slate-200 w-40">
                        <input type="number" step="0.01" value={tempSlide} onChange={(e) => setTempSlide(parseFloat(e.target.value))} disabled={isBusy} className="w-full bg-transparent px-2 text-center font-bold text-slate-700 text-sm focus:outline-none"/>
                        <button onClick={handleManualSetSlide} disabled={isBusy} className="bg-white px-3 py-1.5 rounded-lg text-[10px] font-bold text-slate-500 shadow-sm border border-slate-100">GO</button>
                    </div>
                </div>
            </div>
            
            <div className="flex-[2] bg-white rounded-[2rem] p-6 shadow-xl border border-slate-100 flex flex-col items-center justify-center relative overflow-hidden">
                <div className="absolute top-6 left-6 flex items-center gap-3">
                    <div className={`p-2 rounded-xl transition-colors ${!isHookLocked ? 'bg-emerald-50 text-emerald-600' : 'bg-slate-100 text-slate-500'}`}>
                        <Target size={20}/>
                    </div>
                    <span className="text-sm font-bold text-slate-600 uppercase">Gripper</span>
                </div>
                
                <div className="flex flex-col items-center gap-4 mt-6">
                    <button onClick={toggleHook} disabled={isBusy}
                        className={`w-32 h-16 rounded-full p-2 transition-colors duration-300 shadow-inner relative 
                        ${!isHookLocked ? 'bg-emerald-500' : 'bg-slate-200'}
                        ${disabledClass}`} 
                    >
                        <div className={`w-12 h-12 bg-white rounded-full shadow-lg transform transition-transform duration-300 flex items-center justify-center absolute top-2 
                            ${!isHookLocked ? 'translate-x-16 left-0' : 'translate-x-0 left-2'}`}
                        >
                            {!isHookLocked ? <Unlock size={20} className="text-emerald-500"/> : <Lock size={20} className="text-slate-400"/>}
                        </div>
                    </button>
                    
                    <span className={`text-xl font-black uppercase tracking-widest transition-colors duration-300 ${!isHookLocked ? 'text-emerald-500' : 'text-slate-300'}`}>
                        {!isHookLocked ? 'RELEASED' : 'LOCKED'}
                    </span>
                </div>
            </div>
        </div>
      </div>

      {/* Footer Status */}
      <div className={`fixed bottom-4 right-6 flex items-center gap-2 opacity-90 px-3 py-1 rounded-full border shadow-sm backdrop-blur transition-all duration-300
          ${isError ? 'bg-red-50 border-red-200 text-red-600' : isBusy ? 'bg-orange-50 border-orange-200 text-orange-600' : 'bg-white/80 border-slate-200'}`}>
         <RefreshCw size={12} className={isBusy ? "animate-spin" : "text-slate-400"}/>
         <span className={`text-[10px] font-bold ${isError ? 'text-red-500' : isBusy ? 'text-orange-500' : 'text-slate-500'}`}>
            SYSTEM STATUS: {status}
         </span>
      </div>

    </div>
  );
};

export default PiggybackControl;