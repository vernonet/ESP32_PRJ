import React from 'react';
import { 
  Zap, 
  Code2, 
  GitCompare, 
  Sliders, 
  Binary, 
  Download, 
  ExternalLink,
  Laptop,
  Cpu,
  BookOpen
} from 'lucide-react';
import { downloadFile } from '../utils/fileDownloader';
import { ENGLISH_CPP, PLATFORMIO_INI } from '../data/sourceCode';

export type ActiveTab = 'simulator' | 'code' | 'diff' | 'arduino' | 'config' | 'protocol' | 'readme';

interface NavbarProps {
  activeTab: ActiveTab;
  setActiveTab: (tab: ActiveTab) => void;
}

export const Navbar: React.FC<NavbarProps> = ({ activeTab, setActiveTab }) => {
  const handleDownloadCpp = () => {
    downloadFile('esp32_JK_BMS.cpp', ENGLISH_CPP);
  };

  return (
    <header className="sticky top-0 z-50 bg-slate-950/85 backdrop-blur-md border-b border-slate-800">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="flex items-center justify-between h-16 gap-4">
          
          {/* Logo & Project Title */}
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 rounded-xl bg-gradient-to-tr from-cyan-600 to-blue-600 flex items-center justify-center text-white shadow-lg shadow-cyan-500/20">
              <Zap className="w-6 h-6" />
            </div>
            <div>
              <div className="flex items-center gap-2">
                <span className="font-bold text-slate-100 text-sm sm:text-base tracking-tight">
                  ESP32 JK-BMS
                </span>
                <span className="px-2 py-0.5 rounded text-[10px] font-bold uppercase tracking-wider bg-emerald-500/20 text-emerald-400 border border-emerald-500/30">
                  English Localized
                </span>
              </div>
              <div className="flex items-center gap-2 text-xs text-slate-400">
                <span>BLE Client + AJAX Web Server</span>
                <span className="text-slate-600">•</span>
                <a 
                  href="https://github.com/vernonet/ESP32_PRJ/tree/main/esp32_jk_bms" 
                  target="_blank" 
                  rel="noreferrer" 
                  className="hover:text-cyan-400 transition-colors flex items-center gap-1"
                >
                  <span>vernonet/ESP32_PRJ</span>
                  <ExternalLink className="w-3 h-3" />
                </a>
              </div>
            </div>
          </div>

          {/* Navigation Tabs */}
          <nav className="hidden md:flex items-center gap-1 bg-slate-900/90 p-1 rounded-xl border border-slate-800 text-xs font-medium">
            <button
              onClick={() => setActiveTab('simulator')}
              className={`px-3 py-1.5 rounded-lg transition-all flex items-center gap-1.5 ${
                activeTab === 'simulator'
                  ? 'bg-cyan-600 text-white font-semibold shadow'
                  : 'text-slate-400 hover:text-slate-200 hover:bg-slate-800'
              }`}
            >
              <Laptop className="w-3.5 h-3.5" />
              <span>Web Simulator</span>
              <span className="w-1.5 h-1.5 rounded-full bg-emerald-400 animate-pulse"></span>
            </button>

            <button
              onClick={() => setActiveTab('code')}
              className={`px-3 py-1.5 rounded-lg transition-all flex items-center gap-1.5 ${
                activeTab === 'code'
                  ? 'bg-cyan-600 text-white font-semibold shadow'
                  : 'text-slate-400 hover:text-slate-200 hover:bg-slate-800'
              }`}
            >
              <Code2 className="w-3.5 h-3.5" />
              <span>English Code</span>
            </button>

            <button
              onClick={() => setActiveTab('diff')}
              className={`px-3 py-1.5 rounded-lg transition-all flex items-center gap-1.5 ${
                activeTab === 'diff'
                  ? 'bg-cyan-600 text-white font-semibold shadow'
                  : 'text-slate-400 hover:text-slate-200 hover:bg-slate-800'
              }`}
            >
              <GitCompare className="w-3.5 h-3.5" />
              <span>114-Line Diff</span>
            </button>

            <button
              onClick={() => setActiveTab('arduino')}
              className={`px-3 py-1.5 rounded-lg transition-all flex items-center gap-1.5 ${
                activeTab === 'arduino'
                  ? 'bg-cyan-600 text-white font-semibold shadow'
                  : 'text-slate-400 hover:text-slate-200 hover:bg-slate-800'
              }`}
            >
              <Cpu className="w-3.5 h-3.5 text-emerald-400" />
              <span>Arduino Guide</span>
            </button>

            <button
              onClick={() => setActiveTab('config')}
              className={`px-3 py-1.5 rounded-lg transition-all flex items-center gap-1.5 ${
                activeTab === 'config'
                  ? 'bg-cyan-600 text-white font-semibold shadow'
                  : 'text-slate-400 hover:text-slate-200 hover:bg-slate-800'
              }`}
            >
              <Sliders className="w-3.5 h-3.5" />
              <span>Configurator</span>
            </button>

            <button
              onClick={() => setActiveTab('protocol')}
              className={`px-3 py-1.5 rounded-lg transition-all flex items-center gap-1.5 ${
                activeTab === 'protocol'
                  ? 'bg-cyan-600 text-white font-semibold shadow'
                  : 'text-slate-400 hover:text-slate-200 hover:bg-slate-800'
              }`}
            >
              <Binary className="w-3.5 h-3.5" />
              <span>Protocol &amp; Fix</span>
            </button>

            <button
              onClick={() => setActiveTab('readme')}
              className={`px-3 py-1.5 rounded-lg transition-all flex items-center gap-1.5 ${
                activeTab === 'readme'
                  ? 'bg-cyan-600 text-white font-semibold shadow'
                  : 'text-slate-400 hover:text-slate-200 hover:bg-slate-800'
              }`}
            >
              <BookOpen className="w-3.5 h-3.5 text-amber-400" />
              <span>README</span>
            </button>
          </nav>

          {/* Quick Download Button */}
          <div className="flex items-center gap-2">
            <button
              onClick={handleDownloadCpp}
              className="px-3.5 py-1.5 bg-gradient-to-r from-cyan-600 to-blue-600 hover:from-cyan-500 hover:to-blue-500 text-white text-xs font-semibold rounded-lg flex items-center gap-1.5 transition shadow-lg shadow-cyan-600/20"
            >
              <Download className="w-3.5 h-3.5" />
              <span className="hidden sm:inline">Download</span>
              <span>.cpp</span>
            </button>
          </div>
        </div>

        {/* Mobile Tab Bar */}
        <div className="flex md:hidden overflow-x-auto py-2 gap-1 border-t border-slate-800/60 no-scrollbar text-xs font-medium">
          <button
            onClick={() => setActiveTab('simulator')}
            className={`px-3 py-1.5 rounded-lg whitespace-nowrap transition ${
              activeTab === 'simulator' ? 'bg-cyan-600 text-white font-semibold' : 'text-slate-400'
            }`}
          >
            Web Simulator
          </button>
          <button
            onClick={() => setActiveTab('code')}
            className={`px-3 py-1.5 rounded-lg whitespace-nowrap transition ${
              activeTab === 'code' ? 'bg-cyan-600 text-white font-semibold' : 'text-slate-400'
            }`}
          >
            English Code
          </button>
          <button
            onClick={() => setActiveTab('diff')}
            className={`px-3 py-1.5 rounded-lg whitespace-nowrap transition ${
              activeTab === 'diff' ? 'bg-cyan-600 text-white font-semibold' : 'text-slate-400'
            }`}
          >
            Diff Viewer (114)
          </button>
          <button
            onClick={() => setActiveTab('arduino')}
            className={`px-3 py-1.5 rounded-lg whitespace-nowrap transition ${
              activeTab === 'arduino' ? 'bg-cyan-600 text-white font-semibold' : 'text-slate-400'
            }`}
          >
            Arduino IDE
          </button>
          <button
            onClick={() => setActiveTab('config')}
            className={`px-3 py-1.5 rounded-lg whitespace-nowrap transition ${
              activeTab === 'config' ? 'bg-cyan-600 text-white font-semibold' : 'text-slate-400'
            }`}
          >
            Configurator
          </button>
          <button
            onClick={() => setActiveTab('protocol')}
            className={`px-3 py-1.5 rounded-lg whitespace-nowrap transition ${
              activeTab === 'protocol' ? 'bg-cyan-600 text-white font-semibold' : 'text-slate-400'
            }`}
          >
            Protocol &amp; Fix
          </button>
          <button
            onClick={() => setActiveTab('readme')}
            className={`px-3 py-1.5 rounded-lg whitespace-nowrap transition ${
              activeTab === 'readme' ? 'bg-cyan-600 text-white font-semibold' : 'text-slate-400'
            }`}
          >
            README
          </button>
        </div>
      </div>
    </header>
  );
};
