import React from 'react';
import { Gamepad2, Layers, Activity } from 'lucide-react';

const Sidebar = ({ activeTab, setActiveTab }) => {
  const menuItems = [
    { id: 'mobile', label: 'Mobile Base', icon: <Gamepad2 size={24} /> },
    { id: 'piggyback', label: 'Piggyback', icon: <Layers size={24} /> },
  ];

  return (
    <div className="w-24 bg-white h-screen flex flex-col items-center py-6 border-r border-slate-200 shadow-[2px_0_10px_rgba(0,0,0,0.02)] z-50">
      {/* Logo Area */}
      <div className="mb-8">
        <div className="w-10 h-10 bg-blue-600 rounded-lg flex items-center justify-center shadow-lg shadow-blue-500/30">
          <Activity size={20} color="white" strokeWidth={3} />
        </div>
      </div>

      {/* Menu Items */}
      <div className="flex flex-col gap-3 w-full px-3">
        {menuItems.map((item) => (
          <button
            key={item.id}
            onClick={() => setActiveTab(item.id)}
            className={`flex flex-col items-center justify-center py-3 rounded-lg transition-all duration-200 group ${
              activeTab === item.id
                ? 'bg-blue-50 text-blue-700 ring-1 ring-blue-200'
                : 'text-slate-400 hover:bg-slate-50 hover:text-slate-600'
            }`}
          >
            <div className={`transition-transform duration-200 ${activeTab === item.id ? '-translate-y-1' : ''}`}>
              {item.icon}
            </div>
            <span className={`text-[10px] font-bold mt-1 ${activeTab === item.id ? 'opacity-100' : 'opacity-0 group-hover:opacity-100'} transition-opacity`}>
              {item.label}
            </span>
          </button>
        ))}
      </div>
    </div>
  );
};

export default Sidebar;