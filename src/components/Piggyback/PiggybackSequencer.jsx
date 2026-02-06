import React, { useState, useEffect } from 'react';
import * as ROSLIB from 'roslib';
import { 
  ArrowLeftRight, Layers, Ruler, PackagePlus, PackageMinus, 
  Play, Activity, Box, Edit3, Check, XOctagon
} from 'lucide-react';

const PiggybackSequencer = ({ ros }) => {
  // --- STATE ---
  const [direction, setDirection] = useState(1); // 1=Left, 2=Right (ตาม CLI: LEFT=1, RIGHT=2)
  const [robotShelf, setRobotShelf] = useState(0); // 0, 1, 2, 3
  const [liftHeight, setLiftHeight] = useState(815.0); 
  const [isEditingHeight, setIsEditingHeight] = useState(false); 
  const [tempHeight, setTempHeight] = useState(815.0);

  // Status จาก Robot
  const [robotStatus, setRobotStatus] = useState({ 
    active_action: 'IDLE', 
    feedback_msg: 'Ready' 
  });

  // --- PRESETS ---
  const HEIGHT_PRESETS = [
    { label: 'LEVEL 4', value: 1100.0, color: 'bg-purple-500' },
    { label: 'LEVEL 3', value: 815.0,  color: 'bg-blue-500' },
    { label: 'LEVEL 2', value: 550.0,  color: 'bg-teal-500' },
    { label: 'LEVEL 1', value: 250.0,  color: 'bg-slate-500' },
  ];

  // --- SUBSCRIBE STATUS ---
  useEffect(() => {
    if (!ros) return;
    const statusListener = new ROSLIB.Topic({
      ros: ros, name: '/web_full_status', messageType: 'std_msgs/String'
    });
    statusListener.subscribe((msg) => {
      try {
        const data = JSON.parse(msg.data);
        setRobotStatus({
          active_action: data.active_action || 'IDLE',
          feedback_msg: data.feedback_msg || ''
        });
      } catch (e) {}
    });
    return () => statusListener.unsubscribe();
  }, [ros]);

  // --- SEND COMMAND (START) ---
  const sendSequenceCommand = (isRetrieving) => {
    if (!ros) return alert("ROS Disconnected!");
    const cmdTopic = new ROSLIB.Topic({
      ros: ros, name: '/web_command_gateway', messageType: 'std_msgs/String'
    });
    
    // ✅ Payload ตรงกับ CLI ที่ให้มา
    const payload = {
      type: 'PIGGYBACK_SEQUENCE',
      turntable_direction: direction, // 1 or 2
      robot_shelves: robotShelf,      // 0 to 3
      lift_height_mm: parseFloat(liftHeight),
      is_retrieving: isRetrieving,    // true or false
      shelf_id: "",                   // Ignore QR
      tote_id: ""                     // Ignore QR
    };
    cmdTopic.publish({ data: JSON.stringify(payload) });
  };

  // --- SEND COMMAND (STOP/CANCEL) ---
  const handleStopAction = () => {
    if (!ros) return;
    const cmdTopic = new ROSLIB.Topic({
      ros: ros, name: '/web_command_gateway', messageType: 'std_msgs/String'
    });
    // ✅ ส่งคำสั่ง Stop เพื่อไป Trigger cancel_goal
    cmdTopic.publish({ data: JSON.stringify({ stop: true, type: 'CANCEL_SEQUENCE' }) });
  };

  const handleSaveHeight = () => {
      setLiftHeight(parseFloat(tempHeight));
      setIsEditingHeight(false);
  };

  const isRunning = robotStatus.active_action === 'SEQUENCE' || robotStatus.active_action === 'TRANSPORT';

  return (
    <div className="flex flex-col h-full bg-slate-50/50 p-4 gap-4 relative">
      
      {/* Header */}
      <div className="flex items-center justify-between px-2 shrink-0">
        <div className="flex items-center gap-2 text-slate-700">
           <div className="p-2 bg-blue-100 text-blue-600 rounded-lg"><Play size={20} fill="currentColor"/></div>
           <div>
             <h2 className="font-black text-lg leading-tight">AUTO SEQUENCER</h2>
             <p className="text-[10px] font-bold text-slate-400">TRANSPORT TOTE v1.5</p>
           </div>
        </div>
        {isRunning && (
            <div className="flex items-center gap-2 px-3 py-1 bg-blue-50 text-blue-600 rounded-full border border-blue-200 animate-pulse">
                <Activity size={14} className="animate-spin"/>
                <span className="text-xs font-bold">RUNNING...</span>
            </div>
        )}
      </div>

      {/* --- MAIN CONTENT GRID --- */}
      <div className={`flex-1 grid grid-cols-2 gap-4 min-h-0 transition-opacity duration-300 ${isRunning ? 'opacity-50 pointer-events-none grayscale' : ''}`}>
          
          {/* 🔴 LEFT COL: ROBOT CONFIG */}
          <div className="bg-white rounded-[1.5rem] p-5 shadow-lg border border-slate-100 flex flex-col gap-6">
              {/* 1. Turntable */}
              <div>
                  <label className="text-[10px] font-black text-slate-400 uppercase mb-3 flex items-center gap-1">
                      <ArrowLeftRight size={12}/> Turntable Direction
                  </label>
                  <div className="flex gap-2">
                      {/* Left = 1 */}
                      <button onClick={() => setDirection(1)} 
                          className={`flex-1 py-4 rounded-xl border-2 font-bold text-sm transition-all active:scale-95 flex flex-col items-center gap-1
                          ${direction === 1 ? 'border-purple-500 bg-purple-50 text-purple-700 shadow-md' : 'border-slate-100 bg-slate-50 text-slate-400 hover:border-purple-200'}`}>
                          <span className="text-xs">LEFT</span>
                          <span className="text-[10px] opacity-50">(Dir 1)</span>
                      </button>
                      {/* Right = 2 */}
                      <button onClick={() => setDirection(2)} 
                          className={`flex-1 py-4 rounded-xl border-2 font-bold text-sm transition-all active:scale-95 flex flex-col items-center gap-1
                          ${direction === 2 ? 'border-purple-500 bg-purple-50 text-purple-700 shadow-md' : 'border-slate-100 bg-slate-50 text-slate-400 hover:border-purple-200'}`}>
                          <span className="text-xs">RIGHT</span>
                          <span className="text-[10px] opacity-50">(Dir 2)</span>
                      </button>
                  </div>
              </div>

              {/* 2. Robot Slot */}
              <div className="flex-1 flex flex-col">
                  <label className="text-[10px] font-black text-slate-400 uppercase mb-3 flex items-center gap-1">
                      <Layers size={12}/> Robot Slot Selection
                  </label>
                  <div className="flex-1 bg-slate-50 rounded-2xl border border-slate-100 p-2 flex flex-col-reverse gap-2">
                      {[0, 1, 2, 3].map((slot) => (
                          <button key={slot} onClick={() => setRobotShelf(slot)}
                              className={`flex-1 flex items-center justify-between px-4 rounded-xl border-2 transition-all active:scale-95
                              ${robotShelf === slot ? 'bg-white border-blue-500 shadow-md text-blue-700' : 'bg-transparent border-transparent text-slate-400 hover:bg-white/50'}`}>
                              <div className="flex items-center gap-3">
                                  <Box size={16} strokeWidth={2.5}/>
                                  <span className="font-bold text-sm">SLOT {slot}</span>
                              </div>
                              {robotShelf === slot && <div className="w-2 h-2 bg-blue-500 rounded-full animate-pulse"/>}
                          </button>
                      ))}
                  </div>
              </div>
          </div>

          {/* 🔵 RIGHT COL: TARGET SHELF */}
          <div className="bg-white rounded-[1.5rem] p-5 shadow-lg border border-slate-100 flex flex-col relative overflow-hidden">
              <label className="text-[10px] font-black text-slate-400 uppercase mb-3 flex items-center gap-1 z-10">
                  <Ruler size={12}/> Target Shelf Height
              </label>

              {/* Height Display & Edit */}
              <div className="flex items-center justify-between mb-4 z-10 bg-slate-50 p-2 rounded-xl border border-slate-100">
                  {isEditingHeight ? (
                      <div className="flex items-center gap-2 w-full">
                          <input type="number" value={tempHeight} onChange={(e)=>setTempHeight(e.target.value)}
                              className="w-full bg-white border border-blue-300 rounded-lg px-2 py-1 text-lg font-black text-center outline-none text-blue-600" autoFocus />
                          <button onClick={handleSaveHeight} className="p-2 bg-green-500 text-white rounded-lg shadow-sm active:scale-95">
                              <Check size={18}/>
                          </button>
                      </div>
                  ) : (
                      <>
                        <div className="flex flex-col px-2">
                            <span className="text-[10px] font-bold text-slate-400">CURRENT TARGET</span>
                            <span className="text-2xl font-black text-slate-800 tabular-nums">{liftHeight.toFixed(0)} <span className="text-xs text-slate-400 font-normal">mm</span></span>
                        </div>
                        <button onClick={() => { setTempHeight(liftHeight); setIsEditingHeight(true); }} 
                            className="p-2 bg-white border border-slate-200 text-slate-400 rounded-lg hover:text-blue-500 hover:border-blue-200 transition-colors">
                            <Edit3 size={18}/>
                        </button>
                      </>
                  )}
              </div>

              {/* Visual Shelf Rack */}
              <div className="flex-1 relative bg-slate-50 rounded-2xl border border-slate-100 flex flex-col justify-between py-4 px-2 overflow-hidden">
                  <div className="absolute inset-0 flex flex-col justify-between py-6 px-4 opacity-10 pointer-events-none">
                     {[...Array(10)].map((_,i)=><div key={i} className="w-full h-px bg-slate-800 border-t border-dashed"/>)}
                  </div>

                  <div className="flex flex-col justify-between h-full z-10 gap-2">
                      {HEIGHT_PRESETS.map((preset) => {
                          const isActive = Math.abs(liftHeight - preset.value) < 10;
                          return (
                              <button key={preset.label} onClick={() => setLiftHeight(preset.value)}
                                  className={`relative w-full py-3 px-4 rounded-xl border-2 text-left transition-all active:scale-95 flex items-center justify-between
                                  ${isActive ? 'bg-white border-slate-800 shadow-lg scale-105 z-20' : 'bg-white/50 border-white text-slate-400 hover:border-blue-200'}`}>
                                  <div>
                                      <div className={`text-[10px] font-black ${isActive ? 'text-slate-800' : 'text-slate-400'}`}>{preset.label}</div>
                                      <div className={`text-xs font-mono font-bold ${isActive ? 'text-blue-600' : 'text-slate-300'}`}>{preset.value} mm</div>
                                  </div>
                                  {isActive && <div className="w-2 h-2 bg-blue-500 rounded-full"/>}
                                  <div className={`absolute left-0 top-1/2 -translate-y-1/2 w-1 h-2/3 rounded-r-full transition-colors ${isActive ? preset.color : 'bg-transparent'}`}/>
                              </button>
                          )
                      })}
                  </div>
              </div>
          </div>
      </div>

      {/* --- FOOTER: ACTION & STOP --- */}
      <div className="shrink-0 pt-2 relative">
           {isRunning ? (
               // 🛑 STOP BUTTON STATE (Overlay)
               <div className="flex gap-4 items-center animate-in fade-in slide-in-from-bottom-4 duration-300">
                   <div className="flex-1 bg-slate-800 text-white p-4 rounded-2xl shadow-lg flex items-center gap-4 animate-pulse">
                        <Activity className="animate-spin text-blue-400" size={24}/>
                        <div className="text-left overflow-hidden">
                            <div className="font-black text-lg tracking-wider whitespace-nowrap">EXECUTING...</div>
                            <div className="text-xs font-mono text-slate-400 truncate w-full">{robotStatus.feedback_msg}</div>
                        </div>
                   </div>
                   
                   {/* ปุ่ม STOP สีแดงใหญ่ๆ */}
                   <button onClick={handleStopAction} className="bg-red-600 text-white p-4 rounded-2xl shadow-lg border-2 border-red-500 hover:bg-red-700 active:scale-95 transition-all flex items-center gap-2">
                        <XOctagon size={32} fill="white" className="text-red-600"/>
                        <div className="text-left leading-none pr-2">
                            <div className="font-black text-xl">STOP</div>
                            <div className="text-[9px] font-bold opacity-80">CANCEL ACTION</div>
                        </div>
                   </button>
               </div>
           ) : (
               // ▶️ START BUTTONS STATE
               <div className="grid grid-cols-2 gap-4 h-20">
                    <button onClick={() => sendSequenceCommand(true)}
                        className="bg-emerald-50 border-2 border-emerald-200 rounded-2xl flex items-center justify-center gap-3 hover:bg-emerald-500 hover:border-emerald-500 hover:text-white hover:shadow-xl hover:shadow-emerald-200 transition-all group active:scale-95 text-emerald-700">
                        <div className="p-2 bg-white rounded-full group-hover:bg-white/20 transition-colors">
                            <PackagePlus size={24} />
                        </div>
                        <div className="text-left">
                            <div className="font-black text-lg leading-none">RETRIEVE</div>
                            <div className="text-[10px] font-bold opacity-60">SHELF ➔ ROBOT</div>
                        </div>
                    </button>

                    <button onClick={() => sendSequenceCommand(false)}
                        className="bg-orange-50 border-2 border-orange-200 rounded-2xl flex items-center justify-center gap-3 hover:bg-orange-500 hover:border-orange-500 hover:text-white hover:shadow-xl hover:shadow-orange-200 transition-all group active:scale-95 text-orange-700">
                        <div className="p-2 bg-white rounded-full group-hover:bg-white/20 transition-colors">
                            <PackageMinus size={24} />
                        </div>
                        <div className="text-left">
                            <div className="font-black text-lg leading-none">DEPOSIT</div>
                            <div className="text-[10px] font-bold opacity-60">ROBOT ➔ SHELF</div>
                        </div>
                    </button>
               </div>
           )}
      </div>

    </div>
  );
};

export default PiggybackSequencer;