import React, { useState, useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib'; 
import { QrCode, Gauge } from 'lucide-react';

// ✅ รับ ros เป็น props (ตัวสุดท้ายแล้ว!)
const RobotStatus = ({ ros }) => {
  // --- STATE เก็บค่าความเร็ว (ค่าเดียว) ---
  const [speed, setSpeed] = useState(0.00);
  const poseListener = useRef(null);

  useEffect(() => {
    // ถ้ายังไม่มีการเชื่อมต่อส่งมา ก็รอไปก่อน
    if (!ros) return;

    try {
      const TopicClass = ROSLIB.Topic || ROSLIB.default?.Topic;

      // --- สร้างตัวดักฟังข้อมูล (ใช้ ros จาก App) ---
      poseListener.current = new TopicClass({
        ros: ros, // ✅ ใช้ Connection กลาง
        name: '/turtle1/pose', 
        messageType: 'turtlesim/msg/Pose'
      });

      // --- รับข้อมูลแล้วอัปเดตเข้าหน้าจอ ---
      poseListener.current.subscribe((message) => {
        setSpeed(message.linear_velocity);
      });

    } catch (err) {
      console.error("Status Init Failed:", err);
    }

    return () => {
      // Cleanup: ยกเลิกการฟังเมื่อ Component ถูกทำลาย หรือ ros เปลี่ยน
      if (poseListener.current) poseListener.current.unsubscribe();
    };
  }, [ros]); // ✅ ทำงานใหม่เมื่อได้รับ ros object

  return (
    <div className="grid grid-cols-2 gap-4 h-full">
      {/* Current QR Status */}
      <div className="bg-slate-800 rounded-2xl p-4 flex items-center gap-4 text-white shadow-lg">
        <div className="bg-white/10 p-3 rounded-xl">
          <QrCode size={32} className="text-yellow-400" />
        </div>
        <div>
          <div className="text-xs text-slate-400 uppercase font-bold">Current Location</div>
          <div className="text-2xl font-mono font-black tracking-wider text-yellow-400">
            QR-A05
          </div>
        </div>
      </div>

      {/* Current Speed Status */}
      <div className="bg-slate-800 rounded-2xl p-4 flex items-center gap-4 text-white shadow-lg">
        <div className="bg-white/10 p-3 rounded-xl">
          {/* เปลี่ยนสีไอคอนเมื่อมีความเร็ว */}
          <Gauge size={32} className={`transition-colors duration-300 ${Math.abs(speed) > 0 ? 'text-green-400' : 'text-cyan-400'}`} />
        </div>
        <div>
          <div className="text-xs text-slate-400 uppercase font-bold">Current Speed</div>
          <div className={`text-2xl font-mono font-black tracking-wider transition-all duration-100 ${Math.abs(speed) > 0 ? 'text-green-400' : 'text-cyan-400'}`}>
            {/* แสดงทศนิยม 2 ตำแหน่ง */}
            {speed.toFixed(2)} <span className="text-sm text-slate-500">m/s</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RobotStatus;