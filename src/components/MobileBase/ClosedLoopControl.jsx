import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';
import { Play, Trash, MapPin, Check, Turtle, Square, CheckCircle2, Disc, Bot, Box, Navigation} from 'lucide-react';

const ClosedLoopControl = ({ ros }) => {
  // --- CONFIG ---
  const gridSize = 5;
  const stopThreshold = 0.2; 
  
  // --- STATE ---
  const [selectedPath, setSelectedPath] = useState([]); 
  const [activeIdx, setActiveIdx] = useState(0); 
  const [isSending, setIsSending] = useState(false);
  const [robotPose, setRobotPose] = useState({ x: 0, y: 0 }); 
  
  const currentGridPos = {
    x: Math.round((robotPose.x - 1) / 2),
    y: Math.round((robotPose.y - 1) / 2)
  };

  const pathPublisher = useRef(null);
  const poseListener = useRef(null);

  // --- 1. SETUP ROS ---
  useEffect(() => {
    if (!ros) return;

    try {
        const TopicClass = ROSLIB.Topic || ROSLIB.default?.Topic;

        pathPublisher.current = new TopicClass({
          ros: ros,
          name: '/navigation/path',
          messageType: 'std_msgs/String'
        });
        pathPublisher.current.advertise();

        poseListener.current = new TopicClass({
          ros: ros,
          name: '/turtle1/pose',
          messageType: 'turtlesim/msg/Pose'
        });

        poseListener.current.subscribe((message) => {
          setRobotPose({ x: message.x, y: message.y });
        });

    } catch (err) {
      console.error("Topic Init Failed:", err);
    }

    return () => {
      if (poseListener.current) poseListener.current.unsubscribe();
      if (pathPublisher.current) pathPublisher.current.unadvertise();
    };
  }, [ros]);

  // --- 2. PROGRESS LOGIC ---
  useEffect(() => {
    if (selectedPath.length > 0 && activeIdx < selectedPath.length) {
        
        const target = selectedPath[activeIdx];
        const targetWorld = gridToWorld(target.x, target.y);

        const dx = robotPose.x - targetWorld.x;
        const dy = robotPose.y - targetWorld.y;
        const distance = Math.sqrt(dx * dx + dy * dy);

        if (distance < stopThreshold) {
            setActiveIdx(prev => prev + 1);
        }
    } else if (selectedPath.length > 0 && activeIdx >= selectedPath.length) {
        setTimeout(handleClear, 500);
    }
  }, [robotPose, selectedPath, activeIdx]);

  // --- 3. HELPER & HANDLERS ---
  const gridToWorld = (gx, gy) => {
    return { x: (gx * 2) + 1, y: (gy * 2) + 1 };
  };

  const togglePoint = (x, y) => {
    if (isSending) return;
    if (x === currentGridPos.x && y === currentGridPos.y) return;

    const existingIndex = selectedPath.findIndex(p => p.x === x && p.y === y);
    if (existingIndex !== -1) {
      const newPath = selectedPath.filter((_, index) => index !== existingIndex);
      setSelectedPath(newPath);
    } else {
      setSelectedPath([...selectedPath, { x, y }]);
    }
  };

  const handleRun = () => {
    if (selectedPath.length === 0) return;
    setIsSending(true);
    setActiveIdx(0);

    const realWorldPath = selectedPath.map(p => {
        const world = gridToWorld(p.x, p.y);
        return [world.x, world.y];
    });

    if (pathPublisher.current) {
        const dataToSend = JSON.stringify(realWorldPath);
        const message = { data: dataToSend };
        pathPublisher.current.publish(message);
        console.log("🚀 Sent Path:", dataToSend);
    }
  };

  const handleStop = () => {
    console.log("🛑 EMERGENCY STOP");
    if (pathPublisher.current) {
        const message = { data: "[]" };
        pathPublisher.current.publish(message);
    }
    setIsSending(false);
  };

  const handleClear = () => {
    setSelectedPath([]); 
    setActiveIdx(0);
    setIsSending(false);
  };

  // --- 4. RENDER GRID ---
  const renderGrid = () => {
    let cells = [];
    for (let y = gridSize - 1; y >= 0; y--) {
      for (let x = 0; x < gridSize; x++) {
        
        const pathIndex = selectedPath.findIndex(p => p.x === x && p.y === y);
        const isVisibleTarget = pathIndex !== -1 && pathIndex >= activeIdx;
        const isRobotHere = currentGridPos.x === x && currentGridPos.y === y;
        const realPos = gridToWorld(x, y);

        cells.push(
          <div 
            key={`${x}-${y}`}
            onClick={() => togglePoint(x, y)}
            className={`
              aspect-square rounded-xl border-2 flex items-center justify-center transition-all duration-300 select-none relative
              ${isRobotHere 
                ? 'bg-green-100 border-green-400 cursor-not-allowed ring-4 ring-green-200 z-10' 
                : isVisibleTarget 
                    ? 'bg-blue-500 border-blue-600 text-white shadow-md cursor-pointer scale-95' 
                    : 'bg-white border-slate-200 text-slate-300 hover:border-blue-300 hover:bg-blue-50 cursor-pointer'
              }
            `}
          >
            <span className={`text-[9px] font-mono absolute bottom-1 right-1 ${isVisibleTarget ? 'text-blue-200' : 'text-slate-300'}`}>
              {realPos.x},{realPos.y}
            </span>

            {isRobotHere ? (
                <div className="flex flex-col items-center animate-pulse">
                    <Box size={26} className="text-slate-700 fill-slate-300" />
                    <span className="text-[9px] font-bold text-green-700 bg-green-200 px-1.5 rounded-full mt-0.5">HERE</span>
                </div>
            ) : isVisibleTarget ? (
              <span className="text-2xl font-bold font-mono">{pathIndex + 1}</span>
            ) : (
              <div className="w-2 h-2 rounded-full bg-slate-100" />
            )}
          </div>
        );
      }
    }
    return cells;
  };

  return (
    <div className="h-full flex flex-col lg:flex-row gap-6">
      
      {/* LEFT: GRID MAP */}
      <div className="flex-1 bg-white rounded-3xl border border-slate-200 p-6 shadow-sm flex flex-col relative overflow-hidden">
        <div className="flex justify-between items-center mb-4">
            <h2 className="text-sm font-bold text-slate-500 flex items-center gap-2">
                <MapPin size={18} /> WAYPOINT SELECTOR
            </h2>
            <div className="text-xs font-mono text-slate-400 flex items-center gap-2">
                <Box size={20} className="text-slate-700 fill-slate-300" />
                <span>REAL: ({robotPose.x.toFixed(1)}, {robotPose.y.toFixed(1)})</span>
            </div>
        </div>
        <div className="flex-1 grid grid-cols-5 gap-3 content-center max-w-md mx-auto w-full">
            {renderGrid()}
        </div>
        <div className="absolute inset-0 opacity-[0.03] bg-[radial-gradient(#000_1px,transparent_1px)] [background-size:16px_16px] pointer-events-none" />
      </div>

      {/* RIGHT: CONTROL PANEL */}
      <div className="lg:w-80 flex flex-col gap-4 shrink-0">
        <div className="flex-1 bg-slate-50 rounded-2xl border border-slate-200 p-4 overflow-hidden flex flex-col">
            <h3 className="text-xs font-bold text-slate-400 uppercase tracking-wider mb-3">Path Queue</h3>
            <div className="flex-1 overflow-y-auto pr-2 space-y-2">
                {selectedPath.length === 0 ? (
                    <div className="h-full flex flex-col items-center justify-center text-slate-300 text-sm italic">
                        <span>No points selected</span>
                    </div>
                ) : (
                    selectedPath.map((p, idx) => {
                        const wp = gridToWorld(p.x, p.y);
                        const isCompleted = idx < activeIdx;
                        
                        return (
                            <div key={idx} className={`p-3 rounded-xl border shadow-sm flex items-center justify-between transition-all duration-300 ${isCompleted ? 'bg-slate-50 border-slate-100 opacity-80' : 'bg-white border-slate-200'}`}>
                                <div className="flex items-center gap-3">
                                    <span className={`w-6 h-6 rounded-full flex items-center justify-center text-xs font-bold ${isCompleted ? 'bg-slate-200 text-slate-400' : 'bg-blue-100 text-blue-600'}`}>
                                        {idx + 1}
                                    </span>
                                    <span className={`font-mono font-bold text-sm ${isCompleted ? 'text-slate-400' : 'text-slate-600'}`}>
                                        TARGET ({wp.x}, {wp.y})
                                    </span>
                                </div>
                                {isCompleted ? <CheckCircle2 size={16} className="text-slate-400" /> : <Check size={16} className="text-green-400" />}
                            </div>
                        );
                    })
                )}
            </div>
        </div>

        <div className="grid grid-cols-2 gap-3">
            <button onClick={handleClear} className="bg-white border-2 border-slate-200 text-slate-500 hover:bg-slate-50 hover:text-slate-700 p-3 rounded-xl font-bold flex flex-col items-center gap-1 transition-all">
                <Trash size={20} /> <span className="text-[10px]">CLEAR</span>
            </button>
            <button onClick={handleStop} className="bg-red-50 border-2 border-red-200 text-red-600 hover:bg-red-500 hover:text-white hover:border-red-500 p-3 rounded-xl font-bold flex flex-col items-center gap-1 transition-all">
                <Square size={20} fill="currentColor" /> <span className="text-[10px]">STOP</span>
            </button>
            <button onClick={handleRun} disabled={selectedPath.length === 0 || isSending} className={`col-span-2 p-4 rounded-xl font-bold flex items-center justify-center gap-3 transition-all shadow-lg text-white ${selectedPath.length === 0 ? 'bg-slate-300 cursor-not-allowed shadow-none' : isSending ? 'bg-green-500 scale-95' : 'bg-blue-600 hover:bg-blue-500 hover:shadow-blue-200 hover:-translate-y-1'}`}>
                {isSending ? <div className="animate-spin text-xl">C</div> : <Play size={24} fill="currentColor" />}
                <span>{isSending ? 'SENDING...' : 'RUN PATH'}</span>
            </button>
        </div>
      </div>
    </div>
  );
};

export default ClosedLoopControl;
