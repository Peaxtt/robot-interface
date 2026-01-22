import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib'; 
import { 
  Activity, Battery, Wifi, Navigation, AlertTriangle, CheckCircle2, Scan, Zap
} from 'lucide-react';

const RobotStatus = ({ ros }) => {
  // --- STATE ---
  const [odom, setOdom] = useState({ x: 0.0, y: 0.0, th: 0.0 });
  const [flags, setFlags] = useState({ ff: 0, fs: 0, fm1: 0, fm2: 0 });
  const [battery, setBattery] = useState(100);
  const [voltage, setVoltage] = useState(24.0);
  const [isEmergency, setIsEmergency] = useState(false);
  const [errorText, setErrorText] = useState("SYSTEM NOMINAL"); 
  const [isConnected, setIsConnected] = useState(false);
  
  // Vision State
  const [qrInfo, setQrInfo] = useState({ visible: false, id: -1, cx: 0, cy: 0, ang: 0 });
  const qrTimeoutRef = useRef(null);
  const watchdogRef = useRef(null);

  // --- HELPER FUNCTIONS ---
  const decodeFF = (ff) => {
    if (ff === 0) return null;
    if (ff & 1) return "OVERHEAT";
    if (ff & 2) return "OVER VOLT";
    if (ff & 4) return "UNDER VOLT";
    if (ff & 8) return "CURRENT SURGE"; 
    if (ff & 16) return "E-STOP";
    return "CONTROLLER FAULT";
  };

  const decodeFM = (fm) => {
      if (fm === 0) return "OK";
      if (fm & 1) return "AMP LMT";
      if (fm & 2) return "STALL";
      if (fm & 8) return "CUTOFF"; 
      return "LIMIT";
  };

  useEffect(() => {
    if (!ros) { setIsConnected(false); return; }

    // 1. BRIDGE STATUS (Heartbeat 10Hz)
    // รับข้อมูลรวมศูนย์: Odom, Battery, Hardware Errors
    const bridgeSub = new ROSLIB.Topic({ 
        ros: ros, 
        name: '/web_full_status', 
        messageType: 'std_msgs/String' 
    });

    bridgeSub.subscribe((msg) => {
        try {
            const data = JSON.parse(msg.data);
            setIsConnected(true);
            
            // Watchdog: ถ้าไม่ได้ update นานๆ ให้ขึ้น Offline
            if(watchdogRef.current) clearTimeout(watchdogRef.current);
            watchdogRef.current = setTimeout(() => setIsConnected(false), 2000);

            // Update States
            setOdom(data.position);
            setFlags(data.hardware);
            setBattery(data.battery);
            setVoltage(data.voltage || 24.0);

            // Check Errors
            const ffText = decodeFF(data.hardware.ff);
            const fm1Text = data.hardware.fm1 !== 0 ? `M1:${decodeFM(data.hardware.fm1)}` : "";
            const fm2Text = data.hardware.fm2 !== 0 ? `M2:${decodeFM(data.hardware.fm2)}` : "";

            if (ffText || data.hardware.fm1 !== 0 || data.hardware.fm2 !== 0) {
                setIsEmergency(true);
                setErrorText(ffText || `${fm1Text} ${fm2Text}`.trim());
            } else {
                setIsEmergency(false);
                setErrorText(data.active_action !== 'IDLE' ? data.active_action : "SYSTEM NOMINAL");
            }

        } catch (e) { console.error("Status Parse Error", e); }
    });

    // 2. VISION RAW DATA (Optional)
    // เพื่อโชว์ว่ากล้องเห็น QR หรือไม่ (Direct Feedback)
    const qrSub = new ROSLIB.Topic({ 
        ros: ros, 
        name: '/lector_floor_node/raw_data', 
        messageType: 'std_msgs/String' 
    });

    qrSub.subscribe((msg) => {
        // Format: "ID:1,cx:0.02,cy:-0.01,ang:0.5"
        const txt = msg.data;
        if (txt.includes("ID:")) {
            const idMatch = txt.match(/ID:(-?\d+)/);
            const cxMatch = txt.match(/cx:([-\d.]+)/);
            const cyMatch = txt.match(/cy:([-\d.]+)/); // cy คือระยะห่างใน Sim
            const angMatch = txt.match(/ang:([-\d.]+)/);

            if (idMatch) {
                setQrInfo({
                    visible: true,
                    id: idMatch[1],
                    cx: cxMatch ? parseFloat(cxMatch[1]).toFixed(3) : 0,
                    cy: cyMatch ? parseFloat(cyMatch[1]).toFixed(3) : 0,
                    ang: angMatch ? parseFloat(angMatch[1]).toFixed(1) : 0
                });

                if (qrTimeoutRef.current) clearTimeout(qrTimeoutRef.current);
                qrTimeoutRef.current = setTimeout(() => setQrInfo(prev => ({ ...prev, visible: false })), 500);
            }
        }
    });

    return () => { 
        bridgeSub.unsubscribe(); 
        qrSub.unsubscribe(); 
        if(watchdogRef.current) clearTimeout(watchdogRef.current);
    };
  }, [ros]);

  // --- RENDER ---
  return (
    <div className="h-full bg-white border border-slate-200 rounded-xl shadow-sm flex items-center px-2 py-1 gap-2 overflow-hidden">
      
      {/* 1. MAIN STATUS BADGE */}
      <div className={`w-64 shrink-0 flex items-center gap-3 p-2 rounded-lg border-l-4 transition-all ${!isConnected ? 'bg-slate-100 border-slate-400' : isEmergency ? 'bg-red-50 border-red-500' : 'bg-slate-800 border-green-500'}`}>
         <div className={`p-2 rounded-full ${isEmergency ? 'bg-red-100 text-red-600 animate-pulse' : 'bg-white/10 text-green-400'}`}>
            {!isConnected ? <Wifi size={20} className="text-slate-400"/> : isEmergency ? <AlertTriangle size={20}/> : <CheckCircle2 size={20}/>}
         </div>
         <div className="flex flex-col overflow-hidden">
            <span className="text-[9px] font-bold text-slate-400 uppercase">System Status</span>
            <span className={`text-sm font-black truncate ${isEmergency ? 'text-red-600' : 'text-white'}`}>
                {!isConnected ? "OFFLINE" : errorText}
            </span>
         </div>
      </div>

      <div className="w-px h-12 bg-slate-100 mx-2"></div>

     {/* 2. ODOMETRY - ปรับช่องไฟใหม่ให้ดูเป็นระเบียบ */}
      <div className="flex-1 flex justify-center items-center gap-8">
          <div className="flex items-center gap-3">
              <div className="p-2 bg-blue-50 text-blue-600 rounded-lg shadow-sm">
                <Navigation size={18}/>
              </div>
              <div className="flex flex-col">
                  <span className="text-[9px] font-bold text-slate-400 tracking-wider">POSITION (m)</span>
                  <div className="flex gap-4 font-mono text-sm font-bold text-slate-700">
                      <div className="flex gap-1"><span className="text-slate-400">X:</span>{odom.x.toFixed(2)}</div>
                      <div className="flex gap-1"><span className="text-slate-400">Y:</span>{odom.y.toFixed(2)}</div>
                  </div>
              </div>
          </div>

          {/* แยกส่วนองศาออกมาให้เด่นขึ้น */}
          <div className="flex flex-col border-l border-slate-100 pl-6">
              <span className="text-[9px] font-bold text-slate-400 tracking-wider">HEADING</span>
              <span className="text-base font-black text-blue-600 font-mono leading-none mt-1">
                  {/* ✅ สูตรแปลง: ขึ้น = 0° (UI Standard) */}
                  { ((90 - odom.th + 360) % 360).toFixed(1) }°
              </span>
          </div>
      </div>

      <div className="w-px h-12 bg-slate-100 mx-2"></div>

      {/* 3. QR VISION STATUS */}
      <div className={`w-48 shrink-0 flex items-center gap-3 p-2 rounded-lg border transition-all ${qrInfo.visible ? 'bg-blue-50 border-blue-200' : 'bg-slate-50 border-transparent'}`}>
          <div className={`p-1.5 rounded-md ${qrInfo.visible ? 'bg-blue-100 text-blue-600' : 'bg-slate-200 text-slate-400'}`}>
              <Scan size={18} className={qrInfo.visible ? 'animate-pulse' : ''}/>
          </div>
          <div className="flex flex-col">
              <span className="text-[9px] font-bold text-slate-400">VISION LINK</span>
              <span className={`text-xs font-bold font-mono ${qrInfo.visible ? 'text-blue-700' : 'text-slate-400'}`}>
                  {qrInfo.visible ? `ID:${qrInfo.id} [${qrInfo.cx}, ${qrInfo.cy}]` : "SEARCHING..."}
              </span>
          </div>
      </div>

      <div className="w-px h-12 bg-slate-100 mx-2"></div>

      {/* 4. MOTORS & BATTERY */}
      <div className="flex items-center gap-4 pr-4">
          <div className="flex flex-col items-end gap-1">
              <div className={`text-[9px] font-bold px-1.5 py-0.5 rounded border ${flags.fm1!==0?'bg-red-50 border-red-200 text-red-600':'bg-slate-50 border-slate-200 text-slate-400'}`}>
                  M1: {decodeFM(flags.fm1)}
              </div>
              <div className={`text-[9px] font-bold px-1.5 py-0.5 rounded border ${flags.fm2!==0?'bg-red-50 border-red-200 text-red-600':'bg-slate-50 border-slate-200 text-slate-400'}`}>
                  M2: {decodeFM(flags.fm2)}
              </div>
          </div>
          
          <div className="flex items-center gap-2 pl-4 border-l border-slate-100">
              <div className="flex flex-col items-end">
                  <span className="text-xl font-black text-slate-700 leading-none">{battery}%</span>
                  <span className="text-[8px] font-bold text-slate-400">{voltage.toFixed(1)}V</span>
              </div>
              <Battery size={28} className={`fill-current ${battery > 20 ? 'text-green-500' : 'text-red-500 animate-pulse'}`} />
          </div>
      </div>

    </div>
  );
};

export default RobotStatus;