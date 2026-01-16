import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';
import { Play, Trash, MapPin, Check, Square, CheckCircle2, Box, Anchor, Crosshair } from 'lucide-react';

const ClosedLoopControl = ({ ros }) => {
  // --- 📏 CONFIG (X คือหน้า-หลัง 0.92ม., Y คือซ้าย-ขวา 0.60ม.) ---
  const gridSize = 5;
  const DIST_X = 0.92; // Forward (+X)
  const DIST_Y = 0.60; // Left (+Y)

  // --- STATE ---
  const [selectedPath, setSelectedPath] = useState([]); 
  const [activeIdx, setActiveIdx] = useState(0); 
  const [isSending, setIsSending] = useState(false);
  const [isSettingSync, setIsSettingSync] = useState(false);
  
  // ตำแหน่งที่โปรแกรมจำไว้ (Virtual Grid)
  const [virtualGridPos, setVirtualGridPos] = useState({ x: 2, y: 2 }); 

  const actionClient = useRef(null);
  const currentGoal = useRef(null);

  useEffect(() => {
    if (!ros) return;
    try {
        const ActionClientClass = ROSLIB.ActionClient || ROSLIB.default?.ActionClient;
        actionClient.current = new ActionClientClass({
            ros: ros,
            serverName: '/navigate_vector',
            actionName: 'amr_interfaces/action/NavigateVector'
        });
    } catch (err) { console.error("Action Client Error:", err); }
  }, [ros]);

  // ✅ 1. ฟังก์ชันคำนวณเวกเตอร์ (X: หน้า, Y: ซ้าย)
  const calculateVector = (from, to) => {
    return {
        // บน Grid: y มากขึ้นคือเลื่อนขึ้น -> ตรงกับหน้าหุ่น (+X)
        x: (to.y - from.y) * DIST_X,
        // บน Grid: x น้อยลงคือเลื่อนไปทางซ้าย -> ตรงกับซ้ายหุ่น (+Y)
        y: (from.x - to.x) * DIST_Y,
        theta: 0.0
    };
  };

  const executeStep = (index, currentPos) => {
    if (index >= selectedPath.length) {
        setIsSending(false);
        setSelectedPath([]);
        return;
    }

    setActiveIdx(index);
    const target = selectedPath[index];
    const goalData = calculateVector(currentPos, target);

    console.log(`🚀 Action [Step ${index + 1}]: x=${goalData.x.toFixed(2)}, y=${goalData.y.toFixed(2)}`);

    const GoalClass = ROSLIB.Goal || ROSLIB.default?.Goal;
    const goal = new GoalClass({
        actionClient: actionClient.current,
        goalMessage: goalData
    });

    currentGoal.current = goal;
    goal.on('result', (result) => {
        if (result.success) {
            setVirtualGridPos({ x: target.x, y: target.y });
            executeStep(index + 1, target);
        } else {
            setIsSending(false);
        }
    });
    goal.send();
  };

  const handleRun = () => {
    if (selectedPath.length === 0 || !actionClient.current) return;
    setIsSending(true);
    setActiveIdx(0);
    executeStep(0, virtualGridPos);
  };

  const handleStop = () => {
    if (currentGoal.current) currentGoal.current.cancel();
    setIsSending(false);
  };

  const handleGridClick = (x, y) => {
    if (isSettingSync) {
        setVirtualGridPos({ x, y });
        setIsSettingSync(false);
        return;
    }
    if (isSending) return;
    if (x === virtualGridPos.x && y === virtualGridPos.y) return;

    const existingIndex = selectedPath.findIndex(p => p.x === x && p.y === y);
    if (existingIndex !== -1) {
      setSelectedPath(selectedPath.filter((_, index) => index !== existingIndex));
    } else {
      setSelectedPath([...selectedPath, { x, y }]);
    }
  };

  const renderGrid = () => {
    let cells = [];
    for (let y = gridSize - 1; y >= 0; y--) {
      for (let x = 0; x < gridSize; x++) {
        const pathIndex = selectedPath.findIndex(p => p.x === x && p.y === y);
        const isRobotHere = virtualGridPos.x === x && virtualGridPos.y === y;
        const isVisibleTarget = pathIndex !== -1 && pathIndex >= activeIdx;

        // คำนวณพิกัดสัมพัทธ์โชว์ในช่อง (X: Forward, Y: Left)
        const relX = (y - virtualGridPos.y) * DIST_X;
        const relY = (virtualGridPos.x - x) * DIST_Y;

        cells.push(
          <div key={`${x}-${y}`} onClick={() => handleGridClick(x, y)}
            className={`aspect-square rounded-xl border-2 flex items-center justify-center transition-all duration-500 relative
              ${isSettingSync ? 'bg-blue-50 border-dashed border-blue-400 cursor-crosshair' : ''}
              ${isRobotHere && !isSettingSync ? 'bg-green-100 border-green-500 ring-4 ring-green-100 z-10' 
                : isVisibleTarget ? 'bg-blue-500 border-blue-600 text-white shadow-lg scale-95' 
                : 'bg-white border-slate-200 text-slate-300 hover:border-blue-300 hover:bg-blue-50'}`}>
            
            <span className="text-[7px] font-mono absolute bottom-1 right-1 opacity-50">
                X:{relX.toFixed(1)} Y:{relY.toFixed(1)}
            </span>

            {isRobotHere && !isSettingSync ? (
                <div className="flex flex-col items-center">
                    <Box size={24} className="text-green-600 fill-green-200" />
                    <span className="text-[8px] font-bold text-green-700 mt-1 uppercase">POS:0,0</span>
                </div>
            ) : isVisibleTarget ? (
              <span className="text-2xl font-bold font-mono">{pathIndex + 1}</span>
            ) : isSettingSync ? (
              <Crosshair size={16} className="text-blue-400 opacity-40" />
            ) : <div className="w-1.5 h-1.5 rounded-full bg-slate-100" />}
          </div>
        );
      }
    }
    return cells;
  };

  return (
    <div className="h-full flex flex-col lg:flex-row gap-6">
      <div className="flex-1 bg-white rounded-3xl border border-slate-200 p-6 shadow-sm flex flex-col relative overflow-hidden">
        <div className="flex justify-between items-center mb-4">
            <h2 className="text-sm font-bold text-slate-500 flex items-center gap-2"><MapPin size={18} /> COORDINATE SYSTEM</h2>
            <button onClick={() => setIsSettingSync(!isSettingSync)}
                className={`flex items-center gap-2 px-4 py-2 rounded-xl text-xs font-bold transition-all
                    ${isSettingSync ? 'bg-blue-500 text-white animate-pulse' : 'bg-slate-800 text-white'}`}>
                <Anchor size={16} /> {isSettingSync ? 'SELECT ROBOT CELL...' : 'SET START POINT'}
            </button>
        </div>
        <div className="flex-1 grid grid-cols-5 gap-3 content-center max-w-md mx-auto w-full">{renderGrid()}</div>
        <div className="mt-4 pt-4 border-t border-slate-100 flex justify-between items-center text-[10px] font-mono text-slate-400">
            <span className="font-bold text-blue-600">FRONT (X+) = 0.92m | LEFT (Y+) = 0.60m</span>
            <span>Virtual Execution Mode</span>
        </div>
      </div>

      <div className="lg:w-80 flex flex-col gap-4 shrink-0">
        <div className="flex-1 bg-slate-50 rounded-2xl border border-slate-200 p-4 overflow-hidden flex flex-col">
            <h3 className="text-xs font-bold text-slate-400 uppercase mb-3 text-center tracking-widest">Execution Queue</h3>
            <div className="flex-1 overflow-y-auto pr-2 space-y-2">
                {selectedPath.length === 0 ? <div className="h-full flex flex-col items-center justify-center text-slate-300 text-sm italic">No points selected</div> :
                    selectedPath.map((p, idx) => {
                        const prev = idx === 0 ? virtualGridPos : selectedPath[idx-1];
                        const vec = calculateVector(prev, p);
                        return (
                            <div key={idx} className={`p-3 rounded-xl border shadow-sm flex items-center justify-between transition-all ${idx < activeIdx ? 'opacity-40 bg-slate-100' : 'bg-white'}`}>
                                <div className="flex items-center gap-3">
                                    <span className={`w-6 h-6 rounded-full flex items-center justify-center text-xs font-bold ${idx < activeIdx ? 'bg-slate-200 text-slate-400' : 'bg-blue-100 text-blue-700'}`}>{idx + 1}</span>
                                    <div className="flex flex-col">
                                        <span className="font-mono font-bold text-[11px] text-slate-600 uppercase">Vector: X:{vec.x.toFixed(2)}, Y:{vec.y.toFixed(2)}</span>
                                        <span className="text-[9px] text-slate-400 font-bold uppercase">Theta: 0.0</span>
                                    </div>
                                </div>
                                {idx < activeIdx ? <CheckCircle2 size={16} className="text-green-500" /> : <Check size={16} className="text-slate-200" />}
                            </div>
                        );
                    })
                }
            </div>
        </div>
        <div className="grid grid-cols-2 gap-3">
            <button onClick={() => {setSelectedPath([]); setActiveIdx(0);}} className="bg-white border-2 border-slate-200 text-slate-500 p-3 rounded-xl font-bold flex flex-col items-center gap-1 hover:bg-slate-50 transition-all"><Trash size={20} /><span className="text-[10px]">CLEAR</span></button>
            <button onClick={handleStop} className="bg-red-50 border-2 border-red-200 text-red-600 p-3 rounded-xl font-bold flex flex-col items-center gap-1 hover:bg-red-500 hover:text-white transition-all"><Square size={20} fill="currentColor" /><span className="text-[10px]">STOP</span></button>
            <button onClick={handleRun} disabled={selectedPath.length === 0 || isSending} 
                className={`col-span-2 p-4 rounded-xl font-bold flex items-center justify-center gap-3 shadow-lg text-white transition-all ${selectedPath.length === 0 ? 'bg-slate-300' : isSending ? 'bg-green-500 scale-95' : 'bg-blue-600 hover:bg-blue-700'}`}>
                {isSending ? <div className="animate-spin text-xl">C</div> : <Play size={24} fill="currentColor" />}
                <span>{isSending ? 'SENDING ACTION...' : 'RUN PATH'}</span>
            </button>
        </div>
      </div>
    </div>
  );
};

export default ClosedLoopControl;