/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import React, { useState } from 'react';
import { Navbar, ActiveTab } from './components/Navbar';
import { Esp32WebSimulator } from './components/Simulator/Esp32WebSimulator';
import { CodeExplorer } from './components/CodeViewer/CodeExplorer';
import { DiffViewer } from './components/DiffViewer/DiffViewer';
import { ArduinoGuide } from './components/ArduinoGuide/ArduinoGuide';
import { FirmwareConfigurator } from './components/Configurator/FirmwareConfigurator';
import { ProtocolDocs } from './components/ProtocolDocs/ProtocolDocs';
import { ReadmeViewer } from './components/ReadmeViewer/ReadmeViewer';
import { 
  CheckCircle2, 
  ExternalLink, 
  Github, 
  Layers, 
  ShieldCheck, 
  Sparkles, 
  Zap, 
  Cpu,
  FileCode,
  Terminal,
  Activity
} from 'lucide-react';
import { downloadFile } from './utils/fileDownloader';
import { ENGLISH_CPP, PLATFORMIO_INI } from './data/sourceCode';

export default function App() {
  const [activeTab, setActiveTab] = useState<ActiveTab>('simulator');

  return (
    <div className="min-h-screen bg-slate-950 text-slate-100 flex flex-col font-sans selection:bg-cyan-500/30 selection:text-cyan-200">
      {/* Top Navigation */}
      <Navbar activeTab={activeTab} setActiveTab={setActiveTab} />

      {/* Main Container */}
      <main className="flex-1 max-w-7xl w-full mx-auto px-4 sm:px-6 lg:px-8 py-6 space-y-6">
        
        {/* Localization & Highlights Banner */}
        <div className="relative overflow-hidden bg-gradient-to-r from-slate-900 via-slate-900/90 to-cyan-950/40 border border-slate-800 rounded-2xl p-5 shadow-2xl">
          <div className="absolute right-0 top-0 -mt-8 -mr-8 w-64 h-64 bg-cyan-500/10 rounded-full blur-3xl pointer-events-none"></div>
          
          <div className="relative z-10 flex flex-col lg:flex-row lg:items-center justify-between gap-4">
            <div className="space-y-1.5">
              <div className="flex flex-wrap items-center gap-2">
                <span className="px-2.5 py-0.5 rounded-full text-xs font-semibold bg-emerald-500/20 text-emerald-300 border border-emerald-500/30 flex items-center gap-1.5">
                  <CheckCircle2 className="w-3.5 h-3.5" />
                  <span>100% English Localized</span>
                </span>
                <span className="px-2.5 py-0.5 rounded-full text-xs font-medium bg-slate-800 text-slate-300 border border-slate-700">
                  ESP32 Jikong (JK) BMS Monitor
                </span>
                <span className="px-2.5 py-0.5 rounded-full text-xs font-mono bg-cyan-950 text-cyan-400 border border-cyan-800/40">
                  0 Cyrillic Characters in Code
                </span>
              </div>
              <h1 className="text-xl sm:text-2xl font-bold tracking-tight text-white">
                ESP32 Jikong BMS Firmware &amp; Live Web Studio
              </h1>
              <p className="text-xs sm:text-sm text-slate-400 max-w-3xl leading-relaxed">
                All menus, comments, serial logs, and HTML user interface elements from <span className="text-cyan-300 font-mono">vernonet/ESP32_PRJ/esp32_jk_bms</span> have been translated from Ukrainian into clean, idiomatic English. Test live in the Web Simulator, review the 114-line diff, or download the customized firmware.
              </p>
            </div>

            {/* Quick feature pill buttons */}
            <div className="flex flex-wrap lg:flex-col gap-2 shrink-0">
              <div className="flex items-center gap-2 text-xs font-mono bg-slate-950/80 px-3 py-1.5 rounded-lg border border-slate-800 text-slate-300">
                <Activity className="w-3.5 h-3.5 text-cyan-400" />
                <span>AJAX / JSON Live Polling</span>
              </div>
              <div className="flex items-center gap-2 text-xs font-mono bg-slate-950/80 px-3 py-1.5 rounded-lg border border-slate-800 text-slate-300">
                <ShieldCheck className="w-3.5 h-3.5 text-emerald-400" />
                <span>CCCD 0x2902 ESP-IDF Workaround</span>
              </div>
            </div>
          </div>
        </div>

        {/* Tab Content Panels */}
        {activeTab === 'simulator' && <Esp32WebSimulator initialLanguage="en" />}
        {activeTab === 'code' && <CodeExplorer />}
        {activeTab === 'diff' && <DiffViewer />}
        {activeTab === 'arduino' && <ArduinoGuide />}
        {activeTab === 'config' && <FirmwareConfigurator />}
        {activeTab === 'protocol' && <ProtocolDocs />}
        {activeTab === 'readme' && <ReadmeViewer />}

      </main>

      {/* Footer */}
      <footer className="mt-auto border-t border-slate-800/80 bg-slate-950/90 py-6 text-xs text-slate-500">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 flex flex-col sm:flex-row items-center justify-between gap-4">
          <div className="flex items-center gap-2">
            <span>ESP32 JK-BMS English Studio</span>
            <span>•</span>
            <span>Based on</span>
            <a 
              href="https://github.com/vernonet/ESP32_PRJ/tree/main/esp32_jk_bms" 
              target="_blank" 
              rel="noreferrer"
              className="text-cyan-400 hover:underline flex items-center gap-1 font-mono"
            >
              <Github className="w-3.5 h-3.5" />
              <span>vernonet/ESP32_PRJ</span>
            </a>
          </div>

          <div className="flex items-center gap-4">
            <button 
              onClick={() => downloadFile('esp32_JK_BMS.cpp', ENGLISH_CPP)}
              className="hover:text-slate-300 transition-colors"
            >
              Download esp32_JK_BMS.cpp
            </button>
            <span>•</span>
            <button 
              onClick={() => downloadFile('platformio.ini', PLATFORMIO_INI)}
              className="hover:text-slate-300 transition-colors"
            >
              platformio.ini
            </button>
            <span>•</span>
            <span className="text-emerald-500/80">100% Localized into English</span>
          </div>
        </div>
      </footer>
    </div>
  );
}
