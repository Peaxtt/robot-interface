import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';
import { 
  Play, Trash, MapPin, CheckCircle2, Box, Anchor, 
  ArrowUp, Square, Route, Flag, Navigation, RotateCw, Lock, FileText
} from 'lucide-react';

const ClosedLoopControl = ({ ros, savedPath, setSavedPath, savedStep, setSavedStep }) => {
  
  // --- Constants & Config ---
  const gridSize = 5;
  const getQrId = (col, row) => (col * 5) + row + 1;

  // --- States ---
  const selectedPath = savedPath || [];
  const setSelectedPath = setSavedPath || (() => {});
  
  const activeIdx = savedStep || 0;
  const setActiveIdx = setSavedStep || (() => {});

  const [isSending, setIsSending] = useState(false);
  const [alignStatus, setAlignStatus] = useState("READY"); 
  const [isSettingSync, setIsSettingSync] = useState(false);
  const [finalHeading, setFinalHeading] = useState(0); 
  
  // Real-time Robot State
  const [currentGrid, setCurrentGrid] = useState({ x: 2, y: 0 }); 
  const [realQrId, setRealQrId] = useState("NO REF"); // ✅ 1. เพิ่ม State เก็บ QR ID จริงๆ
  const [robotHeadingDeg, setRobotHeadingDeg] = useState(0); // ✅ 2. องศาจริงของหุ่น (0-360)

  // Mission Summary State
  const [showSummary, setShowSummary] = useState(false); // ✅ 4. State โชว์สรุป

  // --- Refs ---
  const bridgeTopic = useRef(null);
  const isStopRequested = useRef(false);
  const pathIdsRef = useRef([]);
  const runStartTime = useRef(0);

  // --- ROS Setup ---
  useEffect(() => {
    if (!ros) return;
    
    bridgeTopic.current = new ROSLIB.Topic({ 
      ros: ros, 
      name: '/web_command_gateway', 
      messageType: 'std_msgs/String' 
    });
    
    // ✅ 1. Subscribe /qr_id โดยตรง เพื่อความชัวร์
    const qrSub = new ROSLIB.Topic({
        ros: ros,
        name: '/qr_id',
        messageType: 'std_msgs/String'
    });
    qrSub.subscribe((msg) => {
        setRealQrId(msg.data); // เก็บค่าดิบไว้โชว์
        
        // อัปเดตตำแหน่ง Grid (ถ้าค่าเป็นตัวเลข)
        const id = parseInt(msg.data);
        if (!isNaN(id)) {
             const r = (id - 1) % 5;
             const c = Math.floor((id - 1) / 5);
             setCurrentGrid({ x: c, y: r });
        }
    });

    // ✅ 2. Subscribe /odom_qr หรือ /odom เพื่อเอาทิศทางจริง
    const odomSub = new ROSLIB.Topic({
        ros: ros,
        name: '/odom_qr', // หรือ /odom ตามที่มี
        messageType: 'nav_msgs/Odometry'
    });
    odomSub.subscribe((msg) => {
        // แปลง Quaternion เป็น Degree
        const q = msg.pose.pose.orientation;
        const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        const rad = Math.atan2(siny_cosp, cosy_cosp);
        let deg = rad * (180 / Math.PI);
        if (deg < 0) deg += 360;
        setRobotHeadingDeg(deg); // 0=Right, 90=Up, 180=Left, 270=Down
    });

    // Bridge Status
    const statusSub = new ROSLIB.Topic({
      ros: ros,
      name: '/web_full_status',
      messageType: 'std_msgs/String'
    });

    statusSub.subscribe((msg) => {
      try {
        const data = JSON.parse(msg.data);
        const action = data.active_action || "IDLE";
        const feedback = data.feedback_msg || "";
        
        const isRobotBusy = action !== "IDLE";
        const isGracePeriod = (Date.now() - runStartTime.current) < 2000;

        if (isRobotBusy || isGracePeriod) {
            setIsSending(true);
            setAlignStatus(feedback || action);
            
            if (feedback && feedback.includes("QR_")) {
                const parts = feedback.split('_');
                const targetIdStr = parts.find(p => !isNaN(parseInt(p)));
                if (targetIdStr) {
                    const targetId = targetIdStr;
                    const idx = pathIdsRef.current.indexOf(targetId);
                    if (idx !== -1) setActiveIdx(idx);
                }
            }
        } else {
            if (isSending) {
                setIsSending(false);
                setAlignStatus("READY");
                
                if (!isStopRequested.current && pathIdsRef.current.length > 0) {
                     setAlignStatus("MISSION COMPLETED");
                     setShowSummary(true); // ✅ 4. จบงานแล้วโชว์สรุป
                }
            }
        }
      } catch (e) { console.error(e); }
    });

    return () => { 
        statusSub.unsubscribe(); 
        qrSub.unsubscribe();
        odomSub.unsubscribe();
    };
  }, [ros, isSending]);

  // --- Handlers ---
  const handleRun = () => { 
    if (selectedPath.length === 0 || isSending) return; 
    
    isStopRequested.current = false; 
    setIsSending(true); 
    setShowSummary(false); // ปิดสรุปเก่าถ้ามี
    runStartTime.current = Date.now();
    setAlignStatus("STARTING...");
    
    const missionIds = selectedPath.map(p => String(getQrId(p.x, p.y)));
    pathIdsRef.current = missionIds;

    const payload = { 
        type: 'EXECUTE_MISSION', 
        path: missionIds
    };
    
    if (bridgeTopic.current) {
        bridgeTopic.current.publish({ data: JSON.stringify(payload) });
    }
    
    setActiveIdx(0);
  };

  const handleStop = () => { 
    isStopRequested.current = true; 
    setIsSending(false);
    pathIdsRef.current = [];
    setActiveIdx(0);
    setAlignStatus("STOPPED");
    setShowSummary(false);
    
    if (bridgeTopic.current) {
        bridgeTopic.current.publish({ data: JSON.stringify({ stop: true }) }); 
    }
  };

  const handleGridClick = (col, row) => {
    if (isSending) return; 
    
    if (isSettingSync) { setCurrentGrid({ x: col, y: row }); setIsSettingSync(false); return; }
    if (col === currentGrid.x && row === currentGrid.y) return;
    
    const existing = selectedPath.findIndex(p => p.x === col && p.y === row);
    if (existing !== -1) setSelectedPath(selectedPath.filter((_, i) => i !== existing));
    else setSelectedPath([...selectedPath, { x: col, y: row }]);
  };

  // --- Helper: Path Calculation for SVG ---
  // ✅ 3. คำนวณเส้นใหม่: เอาแค่จุดที่ยังไม่เดิน (ตั้งแต activeIdx ขึ้นไป) และเชื่อมกับตำแหน่งหุ่นปัจจุบัน
  const getPolylinePoints = () => {
      const remainingPath = selectedPath.slice(activeIdx);
      if (remainingPath.length === 0) return "";
      
      // จุดเริ่มคือตำแหน่งหุ่นปัจจุบัน
      const startPoint = `${currentGrid.x * 20 + 10} ${100 - (currentGrid.y * 20 + 10)}`;
      
      // จุดต่อๆ ไปใน Path
      const pathPoints = remainingPath.map(p => `${p.x * 20 + 10} ${100 - (p.y * 20 + 10)}`).join(' ');
      
      return `${startPoint} ${pathPoints}`;
  };

  // --- Render ---
  const renderGrid = () => {
    let cells = [];
    for (let row = gridSize - 1; row >= 0; row--) {
      for (let col = 0; col < gridSize; col++) {
        const isRobot = col === currentGrid.x && row === currentGrid.y;
        const pathIdx = selectedPath.findIndex(p => p.x === col && p.y === row);
        const isVisiblePoint = pathIdx !== -1; 
        const isPassed = isSending && pathIdx < activeIdx;
        const isLast = isVisiblePoint && pathIdx === selectedPath.length - 1;
        
        // ✅ 2. หมุนกล่องตาม robotHeadingDeg จริงๆ (ลบ 90 เพราะ 0 องศาของ CSS คือทิศขวา แต่ลูกศรชี้ขึ้น)
        // หรือถ้าลูกศรใน icon ชี้ขึ้น (Up) และ 0 องศา ROS คือขวา (Right) -> ต้อง -90
        // แต่เดี๋ยวก่อน! 
        // ROS: 0=Right, 90=Up
        // CSS rotate(0deg): ปกติ (หัวชี้ขึ้นถ้าใช้ ArrowUp หรือ Box ปกติ)
        // ดังนั้น: rotate = 90 - ros_deg
        const visualDeg = 90 - robotHeadingDeg;

        cells.push(
          <div key={`${col}-${row}`} onClick={() => handleGridClick(col, row)}
            style={{ aspectRatio: '0.61/0.92' }} 
            className={`rounded-xl border flex items-center justify-center relative transition-all duration-300
            ${isSending ? 'cursor-not-allowed' : 'cursor-pointer'} 
            ${isRobot ? 'bg-green-50 border-green-500 ring-2 ring-green-100 z-30 shadow-md scale-105' 
            : isPassed ? 'bg-slate-100 border-slate-200 opacity-50 grayscale' 
            : isVisiblePoint ? 'bg-blue-400/10 border-blue-200 z-10' 
            : isSettingSync ? 'bg-blue-50/50 border-blue-400 border-dashed animate-pulse' 
            : 'bg-white border-slate-100 hover:bg-slate-50'}`}>
            
            <span className="absolute top-1 left-1 text-[7px] opacity-20 font-mono font-bold">Q{getQrId(col,row)}</span>
            
            {isRobot ? (
                <div style={{transform: `rotate(${visualDeg}deg)`}} className="transition-transform duration-300 ease-out">
                    <Box size={24} className="text-green-600 fill-green-100"/>
                    <div className="absolute -top-3 left-1/2 -translate-x-1/2 w-0 h-0 border-l-[5px] border-l-transparent border-r-[5px] border-r-transparent border-b-[8px] border-b-green-600"/>
                </div>
            ) : isVisiblePoint ? (
                <div className="relative flex items-center justify-center w-full h-full">
                    <span className={`text-2xl font-black font-mono ${isPassed ? 'text-slate-300' : 'text-blue-300'}`}>{pathIdx + 1}</span>
                    {isLast && (
                        <div onClick={(e) => { e.stopPropagation(); if(!isSending) setFinalHeading((prev) => (prev + 90) % 360); }} 
                             className={`absolute -top-2 -right-2 bg-orange-500 text-white p-1 rounded-full shadow-lg z-40 border-2 border-white ${isSending ? 'opacity-50' : 'animate-bounce cursor-pointer'}`}>
                            <ArrowUp size={14} style={{ transform: `rotate(${finalHeading}deg)` }} />
                        </div>
                    )}
                </div>
            ) : <div className="w-1.5 h-1.5 rounded-full bg-slate-200"/>}
          </div>
        );
      }
    }
    return cells;
  };

  return (
    <div className="bg-white p-4 lg:p-6 rounded-2xl border border-slate-200 shadow-sm h-full flex gap-6 select-none overflow-hidden relative">
      <style>{`
        @keyframes dashflow { from { stroke-dashoffset: 20; } to { stroke-dashoffset: 0; } }
        .flowing-path { stroke-dasharray: 12, 8; animation: dashflow 1s linear infinite; will-change: stroke-dashoffset; }
      `}</style>

      {/* ✅ 4. MISSION SUMMARY OVERLAY */}
      {showSummary && (
          <div className="absolute inset-0 z-50 bg-white/90 backdrop-blur-sm flex flex-col items-center justify-center animate-in fade-in zoom-in duration-300">
              <div className="bg-white p-8 rounded-3xl shadow-2xl border border-blue-100 flex flex-col items-center max-w-md w-full">
                  <div className="w-20 h-20 bg-green-100 rounded-full flex items-center justify-center mb-4">
                      <CheckCircle2 size={48} className="text-green-600" />
                  </div>
                  <h2 className="text-2xl font-black text-slate-800 mb-2">MISSION COMPLETE!</h2>
                  <p className="text-slate-500 mb-6 text-center">Robot has successfully navigated through all waypoints.</p>
                  
                  <div className="w-full bg-slate-50 p-4 rounded-xl border border-slate-100 mb-6">
                      <div className="flex items-center gap-2 mb-3 text-slate-400 text-xs font-bold uppercase tracking-wider">
                          <Route size={14}/> Path Traveled
                      </div>
                      <div className="flex flex-wrap gap-2">
                          {selectedPath.map((p, i) => (
                              <div key={i} className="flex items-center">
                                  <span className="bg-blue-600 text-white px-2 py-1 rounded text-xs font-bold">Q{getQrId(p.x, p.y)}</span>
                                  {i < selectedPath.length - 1 && <div className="w-4 h-0.5 bg-slate-300 mx-1"></div>}
                              </div>
                          ))}
                      </div>
                  </div>

                  <div className="flex gap-3 w-full">
                      <button onClick={() => setShowSummary(false)} className="flex-1 py-3 rounded-xl border border-slate-200 text-slate-600 font-bold hover:bg-slate-50">CLOSE</button>
                      <button onClick={() => {setSelectedPath([]); setActiveIdx(0); setShowSummary(false);}} className="flex-1 py-3 rounded-xl bg-blue-600 text-white font-bold hover:bg-blue-700 shadow-lg shadow-blue-200">NEW MISSION</button>
                  </div>
              </div>
          </div>
      )}

      {/* LEFT: MAP */}
      <div className="flex-1 flex flex-col relative min-w-0">
        
        {/* HEADER */}
        <div className="flex justify-between items-start mb-4 shrink-0">
            <div>
                <div className="flex items-center gap-2 text-slate-700 mb-1">
                    <div className="p-1.5 bg-blue-50 rounded-lg text-blue-600"><MapPin size={20}/></div>
                    <h2 className="text-lg font-bold tracking-tight uppercase">Mission Control</h2>
                </div>
                <p className="text-[10px] text-slate-400 ml-10 font-bold uppercase tracking-wider">Bridge System Active</p>
            </div>
            
            <button 
                disabled={isSending}
                onClick={() => setIsSettingSync(!isSettingSync)} 
                className={`flex items-center gap-2 px-4 py-2.5 rounded-xl text-xs font-bold border transition-all shadow-sm
                ${isSending ? 'opacity-40 cursor-not-allowed bg-slate-50' : ''}
                ${isSettingSync ? 'bg-blue-600 text-white border-blue-700 animate-pulse ring-4 ring-blue-100' : 'bg-white text-slate-600 hover:bg-slate-50 border-slate-200'}`}>
                {isSending ? <Lock size={14}/> : <Anchor size={16} />} 
                {isSettingSync ? 'SELECT ON MAP...' : 'SET START POS'}
            </button>
        </div>

        {/* MAP CONTAINER */}
        <div className={`flex-1 bg-slate-50/50 rounded-3xl border border-slate-100 flex items-center justify-center p-4 relative overflow-hidden shadow-inner transition-all min-h-0 ${isSending ? 'brightness-95 pointer-events-none' : ''}`}>
            
            <div className="relative h-full w-auto aspect-[0.66] max-w-full">
                
                <svg className="absolute inset-0 pointer-events-none w-full h-full z-20" viewBox="0 0 100 100" preserveAspectRatio="none" style={{ overflow: 'visible' }}>
                   {selectedPath.length > 0 && (
                     /* ✅ 3. เส้นวาดเฉพาะส่วนที่เหลือ (Future Path) */
                     <polyline points={getPolylinePoints()}
                        fill="none" 
                        stroke={isSending ? "#f97316" : "#3b82f6"} 
                        strokeWidth="3.5" vectorEffect="non-scaling-stroke" strokeLinecap="round" strokeLinejoin="round" 
                        className="flowing-path" 
                        opacity="0.8"
                     />
                   )}
                </svg>

                <div className="grid grid-cols-5 gap-3 relative h-full">{renderGrid()}</div>
            </div>
            
            {isSending && (
               <div className="absolute top-4 right-4 bg-orange-100 text-orange-600 px-3 py-1 rounded-full text-[10px] font-bold flex items-center gap-2 border border-orange-200 shadow-sm animate-pulse z-50">
                  <Lock size={12}/> CONTROLS LOCKED
               </div>
            )}
        </div>
        
        {/* FOOTER */}
        <div className="absolute bottom-4 left-6 right-6 flex justify-between items-center text-[12px] font-mono text-slate-400 bg-white/95 backdrop-blur p-2.5 rounded-xl border border-slate-100 shadow-sm z-30 shrink-0">
            <div className="flex gap-4">
                {/* ✅ 1. โชว์ QR ID จริงๆ ที่ได้จาก Topic */}
                <span>QR-REF: <b className="text-purple-600 font-black">{realQrId}</b></span>
                <span>POS: <b className="text-slate-600">({currentGrid.x}, {currentGrid.y})</b></span>
                <span>HDG: <b className="text-blue-600 font-black">{robotHeadingDeg.toFixed(0)}°</b></span>
            </div>
            <div className="flex gap-4">
                <span>STATUS: <b className={isSending ? "text-orange-500 animate-pulse" : "text-green-500"}>
                  {alignStatus}
                </b></span>
                <span>MODE: <b className="text-blue-500 font-black">{isSending ? "AUTO MISSION" : "MANUAL"}</b></span>
            </div>
        </div>
      </div>

      {/* RIGHT: CONTROLS */}
      <div className="w-80 flex flex-col gap-4 shrink-0">
        <div className={`flex-1 bg-slate-50 rounded-2xl border border-slate-200 p-4 flex flex-col shadow-inner overflow-hidden transition-all ${isSending ? 'opacity-60 grayscale pointer-events-none' : ''}`}>
            <div className="flex items-center justify-between mb-4 pb-3 border-b border-slate-200 shrink-0">
                <div className="flex items-center gap-2 text-slate-600 font-bold uppercase tracking-widest text-[10px]"><Route size={16}/> Sequence list</div>
                <span className="text-[10px] font-bold bg-white px-2 py-0.5 rounded-full border border-slate-200 text-slate-400">{selectedPath.length} PTS</span>
            </div>
            <div className="flex-1 overflow-y-auto space-y-2 pr-1">
                {selectedPath.length === 0 ? (
                    <div className="h-full flex flex-col items-center justify-center text-slate-300 gap-2 opacity-50 italic text-xs"><Flag size={24}/> No path defined</div>
                ) : (
                    selectedPath.map((p, i) => (
                        <div key={i} className={`p-3 rounded-xl border flex justify-between items-center transition-all 
                            ${i < activeIdx ? 'bg-slate-50 border-transparent opacity-30 grayscale'  /* ✅ 3. ถ้าผ่านแล้วให้จางลงมากๆ */
                            : i === activeIdx && isSending ? 'bg-blue-600 text-white border-blue-700 shadow-md scale-[1.02] z-10' : 'bg-white border-slate-200 shadow-sm'}`}>
                            <div className="flex items-center gap-3">
                                <span className={`w-6 h-6 rounded-full flex items-center justify-center text-[10px] font-bold ${i === activeIdx && isSending ? 'bg-white/20 text-white' : 'bg-blue-50 text-blue-600'}`}>{i+1}</span>
                                <span className="text-[11px] font-bold uppercase tracking-tighter">Target Q{getQrId(p.x, p.y)}</span>
                            </div>
                            {i < activeIdx ? <CheckCircle2 size={16} className="text-green-500" /> : i === selectedPath.length - 1 && <Navigation size={14} className={i === activeIdx && isSending ? 'text-white' : 'text-orange-500'} style={{transform: `rotate(${finalHeading}deg)`}}/>}
                        </div>
                    ))
                )}
            </div>
        </div>
        
        <div className="flex flex-col gap-2 shrink-0">
            <div className="grid grid-cols-2 gap-2">
                <button 
                    disabled={isSending}
                    onClick={() => {setSelectedPath([]); setActiveIdx(0);}} 
                    className={`bg-white border-2 border-slate-200 text-slate-400 py-3 rounded-xl font-bold flex flex-col items-center gap-1 hover:text-red-500 transition-all shadow-sm ${isSending ? 'opacity-30 cursor-not-allowed' : ''}`}>
                    <Trash size={18}/><span className="text-[9px]">CLEAR</span>
                </button>
                <button onClick={handleStop} className="bg-white border-2 border-red-100 text-red-500 py-3 rounded-xl font-bold flex flex-col items-center gap-1 hover:bg-red-600 hover:text-white transition-all shadow-sm">
                    <Square size={18} fill="currentColor"/><span className="text-[9px]">STOP</span>
                </button>
            </div>
            
            <button onClick={handleRun} disabled={selectedPath.length === 0 || isSending} 
                className={`py-4 rounded-xl font-bold flex items-center justify-center gap-3 shadow-lg text-white transition-all uppercase text-xs tracking-widest 
                ${selectedPath.length === 0 ? 'bg-slate-300' : isSending ? 'bg-orange-500 ring-4 ring-orange-100 cursor-not-allowed' : 'bg-blue-600 hover:bg-blue-700'}`}>
                {isSending ? <RotateCw className="animate-spin" size={20}/> : <Play size={20} fill="currentColor" />}
                <span>{isSending ? alignStatus : 'Run Mission'}</span>
            </button>
        </div>
      </div>
    </div>
  );
};

export default ClosedLoopControl;