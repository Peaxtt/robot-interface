import React from 'react';
import { Gamepad2, Box, Settings, Activity } from 'lucide-react';

const Sidebar = ({ activeTab, setActiveTab }) => {
  const menuItems = [
    { id: 'mobile', icon: <Gamepad2 size={24} />, label: 'Mobile Base' },
    { id: 'piggyback', icon: <Box size={24} />, label: 'Piggyback' },
    { id: 'settings', icon: <Settings size={24} />, label: 'Settings' },
  ];

  return (
    <div className="w-20 bg-white border-r border-slate-200 flex flex-col items-center py-6 gap-6 h-screen shadow-sm z-20">
      {/* LOGO */}
      <div className="w-10 h-10 bg-blue-600 rounded-xl flex items-center justify-center text-white shadow-blue-200 shadow-lg mb-4">
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
  );
};

export default Sidebar;