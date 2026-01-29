import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib'; 
import { 
  Activity, Battery, Wifi, Navigation, AlertTriangle, CheckCircle2, Scan, Zap, Crosshair, AlertOctagon
} from 'lucide-react';

const RobotStatus = ({ ros }) => {
  // --- STATE ---
  const [odom, setOdom] = useState({ x: 0.0, y: 0.0, th: 0.0 });
  const [flags, setFlags] = useState({ ff: 0, fs: 0, fm1: 0, fm2: 0 });
  const [battery, setBattery] = useState(100);
  const [voltage, setVoltage] = useState(24.0);
  
  // Status States
  const [isEmergency, setIsEmergency] = useState(false);
  const [isWarning, setIsWarning] = useState(false); // ✅ สถานะเตือนสีเหลือง (Launch Check)
  const [errorText, setErrorText] = useState("SYSTEM NOMINAL"); 
  const [isConnected, setIsConnected] = useState(false);
  const [lastHeartbeat, setLastHeartbeat] = useState(0); // ✅ จับเวลา Heartbeat ล่าสุด
  
  // Vision & Localization State
  const [qrRaw, setQrRaw] = useState({ visible: false, id: -1 }); 
  const [confirmedQrId, setConfirmedQrId] = useState(null);       
  const [qrOdom, setQrOdom] = useState({ x: 0, y: 0, th: 0 });    
  
  const watchdogRef = useRef(null);
  const qrTimeoutRef = useRef(null);

  // --- HELPER FUNCTIONS ---
  const getYawFromQuat = (q) => {
    if (!q) return 0;
    const { x, y, z, w } = q;
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    return Math.atan2(siny_cosp, cosy_cosp);
  };

  const decodeFF = (ff) => {
    if (ff === 0) return null;
    if (ff & 1) return "OVERHEAT";
    if (ff & 2) return "OVER VOLT";
    if (ff & 4) return "BREAK CANCLE";
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

  // ✅ CHECK SYSTEM HEALTH (Launch Check)
  useEffect(() => {
      const healthCheck = setInterval(() => {
          const now = Date.now();
          
          if (!isConnected) {
              // Case 1: ROS Disconnected (Red)
              setIsEmergency(true);
              setIsWarning(false);
              setErrorText("ROS DISCONNECTED");
          } else if (now - lastHeartbeat > 3000) {
              // Case 2: Bridge เงียบเกิน 3 วิ (Yellow) -> ลืม Launch Action Bridge?
              setIsEmergency(false);
              setIsWarning(true);
              setErrorText("CHECK BRIDGE NODE");
          } else if (odom.x === 0 && odom.y === 0 && battery === 0) {
              // Case 3: Bridge มา แต่ค่าเป็น 0 หมด (Yellow) -> ลืม Launch Driver/System Bringup?
              setIsEmergency(false);
              setIsWarning(true);
              setErrorText("CHECK SYSTEM LAUNCH");
          } else if (!isEmergency) {
              // Case 4: Normal (Green/Blue)
              setIsWarning(false);
          }
      }, 1000);
      return () => clearInterval(healthCheck);
  }, [isConnected, lastHeartbeat, odom, battery, isEmergency]);


  useEffect(() => {
    if (!ros) { setIsConnected(false); return; }

    // 1. BRIDGE STATUS (Heartbeat)
    const bridgeSub = new ROSLIB.Topic({ 
        ros: ros, name: '/web_full_status', messageType: 'std_msgs/String' 
    });

    bridgeSub.subscribe((msg) => {
        try {
            const data = JSON.parse(msg.data);
            setIsConnected(true);
            setLastHeartbeat(Date.now()); // Update Heartbeat time

            if(watchdogRef.current) clearTimeout(watchdogRef.current);
            watchdogRef.current = setTimeout(() => setIsConnected(false), 5000);

            setOdom(data.position || { x: 0, y: 0, th: 0 });
            setFlags(data.hardware || { ff: 0, fs: 0, fm1: 0, fm2: 0 });
            setBattery(data.battery || 0);
            setVoltage(data.voltage || 0);

            // Error Logic
            const ffText = decodeFF(data.hardware?.ff || 0);
            const fm1Text = data.hardware?.fm1 ? `M1:${decodeFM(data.hardware.fm1)}` : "";
            const fm2Text = data.hardware?.fm2 ? `M2:${decodeFM(data.hardware.fm2)}` : "";

            if (ffText || data.hardware?.fm1 || data.hardware?.fm2) {
                // Real Error (Red)
                setIsEmergency(true);
                setIsWarning(false);
                setErrorText(ffText || `${fm1Text} ${fm2Text}`.trim());
            } else {
                // No Error (Reset Red, Yellow handled by healthCheck effect)
                setIsEmergency(false);
                // Only update text if not in warning state
                if (!isWarning) {
                    setErrorText(data.active_action !== 'IDLE' ? data.active_action : "SYSTEM NOMINAL");
                }
            }
        } catch (e) { console.error("Status Error", e); }
    });

    // 2. CONFIRMED QR ID
    const qrIdSub = new ROSLIB.Topic({ ros: ros, name: '/qr_id', messageType: 'std_msgs/Int32' });
    qrIdSub.subscribe((msg) => setConfirmedQrId(msg.data));

    // 3. QR ODOMETRY
    const qrOdomSub = new ROSLIB.Topic({ ros: ros, name: '/odom_qr', messageType: 'nav_msgs/Odometry' });
    qrOdomSub.subscribe((msg) => {
        const p = msg.pose.pose.position;
        const q = msg.pose.pose.orientation;
        setQrOdom({ x: p.x, y: p.y, th: getYawFromQuat(q) });
    });

    // 4. RAW VISION
    const qrRawSub = new ROSLIB.Topic({ ros: ros, name: '/lector_floor_node/raw_data', messageType: 'std_msgs/String' });
    qrRawSub.subscribe((msg) => {
        const txt = msg.data;
        if (txt.includes("ID:")) {
            const idMatch = txt.match(/ID:(-?\d+)/);
            if (idMatch) {
                setQrRaw({ visible: true, id: idMatch[1] });
                if (qrTimeoutRef.current) clearTimeout(qrTimeoutRef.current);
                qrTimeoutRef.current = setTimeout(() => setQrRaw({ visible: false, id: -1 }), 500);
            }
        }
    });

    return () => { 
        bridgeSub.unsubscribe(); 
        qrIdSub.unsubscribe();
        qrOdomSub.unsubscribe();
        qrRawSub.unsubscribe();
        if(watchdogRef.current) clearTimeout(watchdogRef.current);
    };
  }, [ros, isWarning]); // Add dependency

  // --- RENDER ---
  return (
    <div className="h-full bg-white border border-slate-200 rounded-xl shadow-sm flex items-center px-2 py-1 gap-2 overflow-hidden select-none">
      
      {/* 1. SYSTEM STATUS (Tri-State Color: Red/Yellow/Green) */}
      <div className={`w-60 shrink-0 flex items-center gap-3 p-2 rounded-lg border-l-4 transition-all duration-500
          ${!isConnected ? 'bg-slate-100 border-slate-400' 
          : isEmergency ? 'bg-red-50 border-red-500' 
          : isWarning ? 'bg-yellow-50 border-yellow-500' // ✅ สีเหลืองเมื่อลืม Launch
          : 'bg-slate-800 border-green-500'}`}>
         
         <div className={`p-2 rounded-full 
            ${!isConnected ? 'text-slate-400' 
            : isEmergency ? 'bg-red-100 text-red-600 animate-pulse' 
            : isWarning ? 'bg-yellow-100 text-yellow-600 animate-bounce' // ✅ Icon เด้งๆ เตือน
            : 'bg-white/10 text-green-400'}`}>
            {!isConnected ? <Wifi size={20}/> 
             : isEmergency ? <AlertTriangle size={20}/> 
             : isWarning ? <AlertOctagon size={20}/> // ✅ Icon เปลี่ยน
             : <CheckCircle2 size={20}/>}
         </div>
         
         <div className="flex flex-col overflow-hidden">
            <span className="text-[9px] font-bold text-slate-400 uppercase">System Status</span>
            <span className={`text-sm font-black truncate 
                ${!isConnected ? 'text-slate-500' 
                : isEmergency ? 'text-red-600' 
                : isWarning ? 'text-yellow-600' // ✅ Text สีเหลือง
                : 'text-white'}`}>
                {!isConnected ? "OFFLINE" : errorText}
            </span>
         </div>
      </div>

      <div className="w-px h-12 bg-slate-100 mx-1"></div>

     {/* 2. ODOMETRY */}
      <div className="flex-1 flex justify-center items-center gap-6">
          <div className="flex items-center gap-3">
              <div className="p-2 bg-blue-50 text-blue-600 rounded-lg shadow-sm">
                <Navigation size={18}/>
              </div>
              <div className="flex flex-col">
                  <span className="text-[9px] font-bold text-slate-400 tracking-wider">ROBOT POSE</span>
                  <div className="flex gap-3 font-mono text-sm font-bold text-slate-700">
                      <div><span className="text-slate-400 text-[10px] mr-1">X</span>{odom.x.toFixed(2)}</div>
                      <div><span className="text-slate-400 text-[10px] mr-1">Y</span>{odom.y.toFixed(2)}</div>
                      <div className="text-blue-600"><span className="text-slate-400 text-[10px] mr-1">H</span>{((90 - odom.th + 360) % 360).toFixed(0)}°</div>
                  </div>
              </div>
          </div>
          
          {/* QR CODE */}
          <div className="flex items-center gap-3 pl-6 border-l border-slate-100">
              <div className={`p-2 rounded-lg transition-colors ${confirmedQrId !== null ? 'bg-purple-100 text-purple-600' : 'bg-slate-50 text-slate-300'}`}>
                <Scan size={18}/>
              </div>
              <div className="flex flex-col">
                  <div className="flex items-center gap-2">
                     <span className="text-[9px] font-bold text-slate-400 tracking-wider">QR LOCALIZATION</span>
                     {qrRaw.visible && <span className="flex h-1.5 w-1.5 rounded-full bg-green-500 animate-pulse"/>}
                  </div>
                  <div className="flex items-center gap-3">
                      <div className={`text-xs font-black px-1.5 py-0.5 rounded ${confirmedQrId!==null ? 'bg-purple-600 text-white' : 'bg-slate-200 text-slate-400'}`}>
                          {confirmedQrId !== null ? `Q${confirmedQrId}` : "NO REF"}
                      </div>
                      <div className="flex gap-2 font-mono text-[10px] font-bold text-slate-500">
                           <span>x:{qrOdom.x.toFixed(2)}</span>
                           <span>y:{qrOdom.y.toFixed(2)}</span>
                      </div>
                  </div>
              </div>
          </div>
      </div>

      <div className="w-px h-12 bg-slate-100 mx-1"></div>

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