import React from 'react';
import { Gamepad2, Box, Settings, Activity, Power } from 'lucide-react';

const Sidebar = ({ activeTab, setActiveTab, onSystemReset }) => {
  const menuItems = [
    { id: 'mobile', icon: <Gamepad2 size={24} />, label: 'Mobile Base' },
    { id: 'piggyback', icon: <Box size={24} />, label: 'Piggyback' },
    // { id: 'settings', icon: <Settings size={24} />, label: 'Settings' }, // Settings ถ้ายังไม่มีฟังก์ชันอื่น ซ่อนไว้ก่อนได้ครับ
  ];

  const confirmReset = () => {
    if (window.confirm("⚠️ DANGER: Are you sure you want to RESET THE SYSTEM?\n\nThis will kill all ROS nodes. You may need to manually relaunch 'run_all.sh' if it doesn't auto-restart.")) {
      onSystemReset();
    }
  };

  return (
    <div className="w-20 bg-white border-r border-slate-200 flex flex-col items-center py-6 h-screen shadow-sm z-20 justify-between">
      
      <div className="flex flex-col items-center gap-6 w-full">
        {/* LOGO */}
        <div className="w-10 h-10 bg-blue-600 rounded-xl flex items-center justify-center text-white shadow-blue-200 shadow-lg">
            <Activity size={24} />
        </div>

        {/* MENU */}
        <div className="flex flex-col gap-4 w-full px-2">
            {menuItems.map((item) => (
            <button
                key={item.id}
                onClick={() => setActiveTab(item.id)}
                className={`w-full aspect-square rounded-xl flex flex-col items-center justify-center gap-1 transition-all duration-200
                ${activeTab === item.id 
                    ? 'bg-blue-50 text-blue-600 shadow-sm ring-2 ring-blue-100' 
                    : 'text-slate-400 hover:bg-slate-50 hover:text-slate-600'}`}
            >
                {item.icon}
                <span className="text-[9px] font-bold">{item.label}</span>
            </button>
            ))}
        </div>
      </div>

      {/* ✅ SYSTEM RESET BUTTON */}
      <div className="w-full px-2 pb-2">
         <button
            onClick={confirmReset}
            className="w-full aspect-square rounded-xl flex flex-col items-center justify-center gap-1 transition-all duration-200 bg-red-50 text-red-500 hover:bg-red-500 hover:text-white border border-red-100 hover:shadow-lg hover:shadow-red-200"
            title="Emergency System Reset"
         >
            <Power size={20} />
            <span className="text-[8px] font-black">RESET</span>
         </button>
      </div>

    </div>
  );
};

export default Sidebar;