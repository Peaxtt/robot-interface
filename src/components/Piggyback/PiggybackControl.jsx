import React, { useState } from 'react';
import { 
  ArrowUp, ArrowDown, ArrowLeft, ArrowRight, RotateCw, RotateCcw,
  Box, Lock, Unlock, Play, Settings, Package, Grip
} from 'lucide-react';

const PiggybackControl = () => {
  // --- STATE ---
  const [liftHeight, setLiftHeight] = useState(0.0);
  const [turnAngle, setTurnAngle] = useState(0);
  const [insertDist, setInsertDist] = useState(0.0);
  const [isHooked, setIsHooked] = useState(false); 

  // Jog Steps
  const [liftStep, setLiftStep] = useState(0.1);
  const [turnStep, setTurnStep] = useState(45);
  const [insertStep, setInsertStep] = useState(0.05);

  // Sequence
  const [seqMode, setSeqMode] = useState('RETRIEVE'); 
  const [seqDir, setSeqDir] = useState('FWD');
  const [isSeqRunning, setIsSeqRunning] = useState(false);

  // --- HANDLERS ---
  const handleIndep = (axis, val) => {
    const cleanVal = parseFloat(val.toFixed(2));
    console.log(`[MANUAL] ${axis} -> ${cleanVal}`);
    return cleanVal;
  };

  const handleSeq = () => {
    setIsSeqRunning(true);
    setTimeout(() => setIsSeqRunning(false), 3000);
  };

  return (
    <div className="flex flex-col lg:flex-row gap-6 h-full select-none font-sans">
      
      {/* 🏗️ LEFT PANEL: TOWER MANUAL CONTROL */}
      <div className="lg:w-7/12 bg-white rounded-3xl border border-slate-200 shadow-sm overflow-hidden flex flex-col">
        <div className="bg-slate-900 text-white p-4 flex justify-between items-center">
          <h2 className="text-xs font-bold uppercase tracking-widest flex items-center gap-2">
            <Settings size={16} className="text-blue-400" /> Manual Tower Control
          </h2>
          <span className="text-[10px] bg-slate-700 px-2 py-1 rounded text-slate-300">JOG MODE</span>
        </div>

        <div className="flex-1 p-6 flex gap-6 overflow-y-auto">
          
          {/* COL 1: LIFT (แนวตั้ง) */}
          <div className="w-1/3 flex flex-col gap-3">
             <div className="text-center">
               <span className="text-[10px] font-bold text-slate-400 uppercase">LIFT HEIGHT</span>
               <div className="text-2xl font-mono font-bold text-blue-600">{liftHeight}m</div>
             </div>
             
             <div className="flex-1 bg-slate-50 rounded-2xl border border-slate-200 p-2 flex flex-col items-center justify-between shadow-inner">
                <button 
                  onClick={() => setLiftHeight(v => handleIndep('LIFT', Math.min(2.0, v + liftStep)))}
                  className="w-full h-20 bg-white rounded-xl shadow-sm border border-slate-200 text-slate-600 hover:text-blue-600 active:bg-blue-50 active:scale-95 transition-all flex items-center justify-center"
                >
                  <ArrowUp size={32} />
                </button>

                <div className="flex flex-col gap-1 w-full my-2">
                   {[0.5, 0.1, 0.01].map(s => (
                     <button 
                       key={s} 
                       onClick={() => setLiftStep(s)}
                       className={`py-1 text-[10px] font-bold rounded ${liftStep === s ? 'bg-blue-600 text-white' : 'text-slate-400 hover:bg-slate-200'}`}
                     >
                       {s}m
                     </button>
                   ))}
                </div>

                <button 
                  onClick={() => setLiftHeight(v => handleIndep('LIFT', Math.max(0, v - liftStep)))}
                  className="w-full h-20 bg-white rounded-xl shadow-sm border border-slate-200 text-slate-600 hover:text-blue-600 active:bg-blue-50 active:scale-95 transition-all flex items-center justify-center"
                >
                  <ArrowDown size={32} />
                </button>
             </div>
          </div>

          {/* COL 2: FORK & HOOK */}
          <div className="flex-1 flex flex-col gap-6">
            
            {/* INSERT */}
            <div className="bg-white border border-slate-200 rounded-2xl p-4 shadow-sm">
               <HeaderLabel label="ARM EXTENSION" value={`${insertDist}m`} icon={<ArrowRight size={14}/>} />
               <div className="flex items-center gap-3 mt-3">
                  <JogButton icon={<ArrowLeft />} onClick={() => setInsertDist(v => handleIndep('INSERT', Math.max(0, v - insertStep)))} />
                  <div className="flex-1 flex justify-center gap-1 bg-slate-100 p-1 rounded-lg">
                     {[0.01, 0.05, 0.1].map(s => (
                       <StepBtn key={s} val={s} current={insertStep} set={setInsertStep} unit="m" />
                     ))}
                  </div>
                  <JogButton icon={<ArrowRight />} onClick={() => setInsertDist(v => handleIndep('INSERT', Math.min(1.0, v + insertStep)))} color="emerald" />
               </div>
            </div>

            {/* ROTATE */}
            <div className="bg-white border border-slate-200 rounded-2xl p-4 shadow-sm">
               <HeaderLabel label="TURNTABLE ANGLE" value={`${turnAngle}°`} icon={<RotateCw size={14}/>} />
               <div className="flex items-center gap-3 mt-3">
                  <JogButton icon={<RotateCcw />} onClick={() => setTurnAngle(v => handleIndep('TURN', v - turnStep))} />
                  <div className="flex-1 flex justify-center gap-1 bg-slate-100 p-1 rounded-lg">
                     {[1, 5, 45, 90].map(s => (
                       <StepBtn key={s} val={s} current={turnStep} set={setTurnStep} unit="°" />
                     ))}
                  </div>
                  <JogButton icon={<RotateCw />} onClick={() => setTurnAngle(v => handleIndep('TURN', v + turnStep))} color="purple" />
               </div>
            </div>

            {/* HOOK */}
            <button 
              onClick={() => { setIsHooked(!isHooked); handleIndep('HOOK', !isHooked); }}
              className={`mt-auto w-full h-20 rounded-2xl flex items-center justify-between px-8 font-bold text-lg transition-all shadow-md active:scale-95 border-2 ${
                isHooked 
                  ? 'bg-orange-50 border-orange-500 text-orange-600' 
                  : 'bg-slate-50 border-slate-200 text-slate-500 hover:bg-slate-100'
              }`}
            >
              <div className="flex items-center gap-3">
                 {isHooked ? <Lock size={28} /> : <Unlock size={28} />}
                 <div className="text-left leading-tight">
                   <div className="text-[10px] uppercase opacity-70">Hook Status</div>
                   <div>{isHooked ? 'LOCKED (GRAB)' : 'UNLOCKED (FREE)'}</div>
                 </div>
              </div>
              <div className={`w-4 h-4 rounded-full ${isHooked ? 'bg-orange-500 animate-pulse' : 'bg-slate-300'}`} />
            </button>

          </div>
        </div>
      </div>

      {/* 📦 RIGHT PANEL: AUTO TASK */}
      <div className="lg:w-5/12 bg-white rounded-3xl border border-slate-200 shadow-sm overflow-hidden flex flex-col">
        <div className="bg-slate-50 p-4 border-b border-slate-200">
           <h2 className="text-xs font-bold text-slate-400 uppercase tracking-widest text-center">Auto Task Sequence</h2>
        </div>

        <div className="flex-1 p-6 flex flex-col gap-6">
           
           <div className="flex gap-4">
              <TaskCard 
                active={seqMode === 'RETRIEVE'} 
                onClick={() => setSeqMode('RETRIEVE')}
                title="GET BOX" 
                sub="Shelf → Robot"
                color="blue"
              />
              <TaskCard 
                active={seqMode === 'PLACE'} 
                onClick={() => setSeqMode('PLACE')}
                title="PUT BOX" 
                sub="Robot → Shelf"
                color="orange"
              />
           </div>

           <div className="bg-slate-50 rounded-2xl p-4 border border-slate-100">
              <div className="text-[10px] font-bold text-slate-400 uppercase text-center mb-3">Target Direction</div>
              <div className="flex gap-2">
                 <DirOption dir="LEFT" icon={<ArrowLeft size={16}/>} active={seqDir === 'LEFT'} onClick={() => setSeqDir('LEFT')} />
                 <DirOption dir="FWD" icon={<ArrowUp size={16}/>} active={seqDir === 'FWD'} onClick={() => setSeqDir('FWD')} />
                 <DirOption dir="RIGHT" icon={<ArrowRight size={16}/>} active={seqDir === 'RIGHT'} onClick={() => setSeqDir('RIGHT')} />
              </div>
           </div>

           <div className="bg-blue-50/50 rounded-xl p-3 border border-blue-100 flex items-center gap-3">
              <Package className="text-blue-500" />
              <div>
                 <div className="text-[10px] font-bold text-blue-400 uppercase">Current Mission</div>
                 <div className="text-sm font-bold text-blue-900">
                    {seqMode} <span className="text-blue-400 mx-1">•</span> {seqDir}
                 </div>
              </div>
           </div>

           <button 
             onClick={handleSeq}
             disabled={isSeqRunning}
             className={`mt-auto w-full h-24 rounded-2xl font-bold text-2xl flex items-center justify-center gap-4 transition-all shadow-lg active:scale-95 ${
               isSeqRunning 
                 ? 'bg-slate-100 text-slate-300 cursor-not-allowed' 
                 : 'bg-green-500 text-white hover:bg-green-600 shadow-green-200'
             }`}
           >
             {isSeqRunning ? <div className="w-8 h-8 border-4 border-white/30 border-t-white rounded-full animate-spin" /> : <Play fill="currentColor" size={32} />}
             {isSeqRunning ? 'WORKING...' : 'START'}
           </button>
        </div>
      </div>

    </div>
  );
};

// --- SUB COMPONENTS ---

const HeaderLabel = ({ label, value, icon }) => (
  <div className="flex justify-between items-end mb-1">
    <div className="flex items-center gap-2 text-[10px] font-bold text-slate-400 uppercase">
       {icon} {label}
    </div>
    <div className="font-mono text-xl font-bold text-slate-700 leading-none">{value}</div>
  </div>
);

const JogButton = ({ icon, onClick, color = "slate" }) => {
  const colors = {
    slate: "bg-white border-slate-200 text-slate-500 hover:border-slate-300",
    emerald: "bg-emerald-50 border-emerald-200 text-emerald-600 hover:border-emerald-300",
    purple: "bg-purple-50 border-purple-200 text-purple-600 hover:border-purple-300",
  };
  return (
    <button onClick={onClick} className={`w-14 h-14 rounded-xl border-2 flex items-center justify-center transition-all active:scale-95 shadow-sm ${colors[color]}`}>
      {icon}
    </button>
  );
};

const StepBtn = ({ val, current, set, unit }) => (
  <button 
    onClick={() => set(val)}
    className={`flex-1 rounded-md text-[10px] font-bold transition-all ${
      current === val ? 'bg-white text-slate-900 shadow-sm border border-slate-200' : 'text-slate-400 hover:bg-white/50'
    }`}
  >
    {val}{unit}
  </button>
);

const TaskCard = ({ active, onClick, title, sub, color }) => {
  const styles = color === 'blue' 
    ? (active ? 'bg-blue-500 text-white shadow-blue-200 border-blue-500' : 'hover:border-blue-200 text-slate-500')
    : (active ? 'bg-orange-500 text-white shadow-orange-200 border-orange-500' : 'hover:border-orange-200 text-slate-500');

  return (
    <button onClick={onClick} className={`flex-1 p-4 rounded-2xl border-2 transition-all shadow-sm active:scale-95 flex flex-col items-center justify-center gap-1 ${active ? 'shadow-lg scale-105 z-10' : 'bg-white border-slate-100' } ${styles}`}>
       {color === 'blue' ? <Grip size={28} /> : <Box size={28} />}
       <span className="font-bold text-sm mt-1">{title}</span>
       <span className={`text-[10px] ${active ? 'opacity-80' : 'opacity-50'}`}>{sub}</span>
    </button>
  );
};

const DirOption = ({ dir, icon, active, onClick }) => (
  <button onClick={onClick} className={`flex-1 py-3 rounded-xl border-2 flex flex-col items-center justify-center gap-1 transition-all ${active ? 'bg-slate-800 border-slate-800 text-white shadow-lg' : 'bg-white border-slate-100 text-slate-300 hover:border-slate-300'}`}>
     {icon}
     <span className="text-[9px] font-bold">{dir}</span>
  </button>
);

export default PiggybackControl;