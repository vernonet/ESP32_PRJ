import React, { useState, useEffect } from 'react';
import { BMSData, BMSSettings, DeviceInfo, ScannedDevice, SimulatorPage } from '../../types/bms';
import { downloadBase64Image } from '../../utils/fileDownloader';
import { REAL_WEB_PNG_BASE64 } from '../../data/embeddedImages';
import { 
  Wifi, 
  Bluetooth, 
  RefreshCw, 
  BatteryCharging, 
  Zap, 
  Cpu, 
  Settings as SettingsIcon, 
  Info as InfoIcon, 
  UploadCloud, 
  Power, 
  CheckCircle2, 
  Sliders, 
  Play, 
  Pause,
  ArrowLeft,
  Search,
  ExternalLink,
  Laptop,
  Download
} from 'lucide-react';

interface Esp32WebSimulatorProps {
  initialLanguage?: 'en' | 'uk';
}

export const Esp32WebSimulator: React.FC<Esp32WebSimulatorProps> = ({ initialLanguage = 'en' }) => {
  const [lang, setLang] = useState<'en' | 'uk'>(initialLanguage);
  const [page, setPage] = useState<SimulatorPage>('dashboard');
  const [isSimRunning, setIsSimRunning] = useState<boolean>(true);
  const [otaProgress, setOtaProgress] = useState<number>(0);
  const [otaState, setOtaState] = useState<'idle' | 'uploading' | 'rebooting' | 'done'>('idle');
  const [otaTimer, setOtaTimer] = useState<number>(10);
  const [saveToast, setSaveToast] = useState<string | null>(null);

  // Initial Simulated BMS State
  const [bmsData, setBmsData] = useState<BMSData>({
    connected: true,
    batteryVoltage: 53.28,
    chargeCurrent: 14.50,
    power: 772.5,
    soc: 82,
    deltaCellVoltage: 0.009,
    temp1: 24.5,
    temp2: 25.1,
    tempMos: 32.4,
    balanceCurrent: 0.650,
    chargeEnabled: true,
    dischargeEnabled: true,
    balanceEnabled: true,
    uptimeSeconds: 148200,
    cycleCount: 42,
    capacityRemain: 229.6,
    nominalCapacity: 280.0,
    cells: Array.from({ length: 16 }, (_, i) => ({
      index: i + 1,
      voltage: +(3.325 + (Math.sin(i * 0.8) * 0.004) + (i === 4 ? 0.008 : 0)).toFixed(3),
      resistance: +(0.0012 + (i % 3) * 0.0001).toFixed(4),
      isBalancing: i === 4 || i === 9,
    })),
  });

  const [settings, setSettings] = useState<BMSSettings>({
    cellUndervoltageProtection: 2.700,
    cellUndervoltageRecovery: 2.900,
    cellOvervoltageProtection: 3.650,
    cellOvervoltageRecovery: 3.550,
    balanceTriggerVoltage: 3.400,
    balanceStartingVoltage: 3.300,
    fullChargeVoltage: 56.800,
    nominalVoltage: 51.200,
    floatChargeVoltage: 54.400,
    balancePrecision: 0.005,
    powerOffVoltage: 41.600,
    maxChargeCurrent: 100.0,
    maxDischargeCurrent: 150.0,
    maxBalanceCurrent: 2.0,
    cellCount: 16,
    totalBatteryCapacity: 280.0,
    bmsMode: 0,
  });

  const [deviceInfo] = useState<DeviceInfo>({
    vendorId: 'JK-BMS-SHENZHEN',
    hardwareVersion: 'HW_V11.XA',
    softwareVersion: 'SW_V11.28W',
    deviceName: 'JK-B2A24S20P',
    devicePasscode: '12345678',
    manufacturingDate: '20241018',
    serialNumber: 'JK2410098421',
    passcode: '1234',
    userData: 'Solar Inverter ESS 48V',
    setupPasscode: '88888888',
    buildTime: 'Nov 07 2026 12:45:00',
  });

  const [scannedDevices] = useState<ScannedDevice[]>([
    { name: 'JK-B2A24S20P', address: 'C8:2B:96:4A:88:12', rssi: -58 },
    { name: 'JK-BD6A20S10P', address: 'E4:65:B8:33:91:04', rssi: -72 },
    { name: 'JK-B1A8S20P', address: '08:D1:F9:67:22:90', rssi: -84 },
  ]);

  // Live simulation ticker
  useEffect(() => {
    if (!isSimRunning || !bmsData.connected) return;

    const interval = setInterval(() => {
      setBmsData(prev => {
        const noise = (Math.random() - 0.5) * 0.05;
        const newCurr = +(prev.chargeEnabled ? Math.max(0, prev.chargeCurrent + noise) : 0).toFixed(2);
        const cellNoise = prev.cells.map(c => ({
          ...c,
          voltage: +(c.voltage + (Math.random() - 0.5) * 0.002).toFixed(3),
        }));
        
        const minV = Math.min(...cellNoise.map(c => c.voltage));
        const maxV = Math.max(...cellNoise.map(c => c.voltage));
        const totalV = +(cellNoise.reduce((acc, c) => acc + c.voltage, 0)).toFixed(2);

        return {
          ...prev,
          batteryVoltage: totalV,
          chargeCurrent: newCurr,
          power: +(totalV * newCurr).toFixed(1),
          deltaCellVoltage: +(maxV - minV).toFixed(3),
          uptimeSeconds: prev.uptimeSeconds + 2,
          cells: cellNoise,
        };
      });
    }, 2000);

    return () => clearInterval(interval);
  }, [isSimRunning, bmsData.connected]);

  // OTA countdown timer
  useEffect(() => {
    if (otaState === 'rebooting') {
      const timer = setInterval(() => {
        setOtaTimer(t => {
          if (t <= 1) {
            clearInterval(timer);
            setOtaState('idle');
            setPage('dashboard');
            return 10;
          }
          return t - 1;
        });
      }, 1000);
      return () => clearInterval(timer);
    }
  }, [otaState]);

  // Toggle Controls
  const toggleCharge = () => {
    setBmsData(d => ({ ...d, chargeEnabled: !d.chargeEnabled }));
  };

  const toggleDischarge = () => {
    setBmsData(d => ({ ...d, dischargeEnabled: !d.dischargeEnabled }));
  };

  const toggleBalance = () => {
    setBmsData(d => ({ 
      ...d, 
      balanceEnabled: !d.balanceEnabled,
      balanceCurrent: !d.balanceEnabled ? 0.650 : 0
    }));
  };

  const handleSimulatedOta = () => {
    setOtaState('uploading');
    setOtaProgress(0);
    const interval = setInterval(() => {
      setOtaProgress(p => {
        if (p >= 100) {
          clearInterval(interval);
          setOtaState('rebooting');
          setOtaTimer(10);
          return 100;
        }
        return p + 25;
      });
    }, 400);
  };

  const handleSettingsSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    setSaveToast(lang === 'en' ? 'Settings saved to BMS registers (0x0A, 0x0C, 0x41, 0x27)!' : 'Налаштування збережено в регістри BMS!');
    setTimeout(() => {
      setSaveToast(null);
      setPage('dashboard');
    }, 1500);
  };

  return (
    <div className="space-y-6">
      {/* Simulation Master Controller Bar */}
      <div className="bg-slate-900 border border-slate-800 rounded-xl p-4 shadow-xl flex flex-wrap items-center justify-between gap-4">
        <div className="flex items-center gap-3">
          <div className="p-2.5 bg-cyan-500/10 border border-cyan-500/30 rounded-lg text-cyan-400">
            <Laptop className="w-5 h-5" />
          </div>
          <div>
            <div className="flex items-center gap-2">
              <h3 className="font-semibold text-slate-100 text-sm md:text-base">
                Interactive ESP32 JK-BMS Web Server Emulator
              </h3>
              <span className="px-2 py-0.5 text-xs font-mono rounded bg-emerald-500/20 text-emerald-400 border border-emerald-500/30">
                Port 8089 Active
              </span>
            </div>
            <p className="text-xs text-slate-400">
              Live simulation of the firmware web server running on the ESP32 with real AJAX endpoints.
            </p>
          </div>
        </div>

        {/* Simulator controls */}
        <div className="flex flex-wrap items-center gap-2">
          {/* Language Toggle */}
          <div className="flex items-center bg-slate-950 p-1 rounded-lg border border-slate-800 text-xs font-medium">
            <span className="text-slate-400 px-2">Firmware UI:</span>
            <button
              onClick={() => setLang('en')}
              className={`px-3 py-1 rounded transition-colors ${
                lang === 'en'
                  ? 'bg-cyan-600 text-white shadow-sm font-semibold'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              English (Localized)
            </button>
            <button
              onClick={() => setLang('uk')}
              className={`px-3 py-1 rounded transition-colors ${
                lang === 'uk'
                  ? 'bg-amber-600 text-white shadow-sm font-semibold'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              Ukrainian (Original)
            </button>
          </div>

          {/* Connection Toggle */}
          <button
            onClick={() => setBmsData(d => ({ ...d, connected: !d.connected }))}
            className={`px-3 py-1.5 rounded-lg text-xs font-medium border flex items-center gap-1.5 transition-all ${
              bmsData.connected
                ? 'bg-emerald-500/10 text-emerald-400 border-emerald-500/30 hover:bg-emerald-500/20'
                : 'bg-rose-500/10 text-rose-400 border-rose-500/30 hover:bg-rose-500/20'
            }`}
          >
            <Bluetooth className="w-3.5 h-3.5" />
            {bmsData.connected ? 'BLE: Connected' : 'BLE: Disconnected'}
          </button>

          {/* Telemetry Ticker Pause/Resume */}
          <button
            onClick={() => setIsSimRunning(r => !r)}
            className="px-3 py-1.5 rounded-lg text-xs font-medium bg-slate-800 text-slate-300 hover:bg-slate-700 border border-slate-700 flex items-center gap-1.5"
            title={isSimRunning ? 'Pause simulation updates' : 'Resume simulation updates'}
          >
            {isSimRunning ? <Pause className="w-3.5 h-3.5 text-amber-400" /> : <Play className="w-3.5 h-3.5 text-emerald-400" />}
            {isSimRunning ? 'Live Stream: Active' : 'Live Stream: Paused'}
          </button>

          {/* GitHub Banner Download */}
          <button
            onClick={() => downloadBase64Image('esp32_jkbms_real_web.png', REAL_WEB_PNG_BASE64, 'image/png')}
            className="px-3 py-1.5 rounded-lg text-xs font-medium bg-purple-600/20 text-purple-300 hover:bg-purple-600/30 border border-purple-500/30 flex items-center gap-1.5 transition shadow cursor-pointer"
            title="Download clean binary PNG screenshot directly from memory for GitHub README"
          >
            <Download className="w-3.5 h-3.5" />
            <span className="hidden sm:inline">GitHub Screenshot</span>
            <span>(.png)</span>
          </button>
        </div>
      </div>

      {/* Browser Frame Replica */}
      <div className="bg-slate-900 border border-slate-800 rounded-xl overflow-hidden shadow-2xl">
        {/* Browser Top Address Bar */}
        <div className="bg-slate-950 px-4 py-2.5 border-b border-slate-800 flex items-center justify-between text-xs text-slate-400">
          <div className="flex items-center gap-2">
            <span className="w-3 h-3 rounded-full bg-rose-500/70 inline-block"></span>
            <span className="w-3 h-3 rounded-full bg-amber-500/70 inline-block"></span>
            <span className="w-3 h-3 rounded-full bg-emerald-500/70 inline-block"></span>
            <span className="ml-2 font-mono text-slate-500 hidden sm:inline">ESP32 Embedded Web Server</span>
          </div>

          <div className="bg-slate-900 border border-slate-800 px-3 py-1 rounded-md text-slate-300 font-mono text-xs flex items-center gap-2 w-full max-w-md mx-4">
            <span className="text-emerald-400">http://</span>
            <span>192.168.1.185:8089{page === 'dashboard' ? '/' : `/${page}`}</span>
          </div>

          <div className="flex items-center gap-3">
            <span className="flex items-center gap-1 text-slate-400">
              <Wifi className="w-3.5 h-3.5 text-cyan-400" />
              <span className="hidden sm:inline">RSSI: -64 dBm</span>
            </span>
          </div>
        </div>

        {/* The Simulated ESP32 Web Page Content */}
        <div className="p-4 sm:p-6 bg-slate-100 text-slate-800 min-h-[520px] font-sans">
          
          {/* Toast Notification */}
          {saveToast && (
            <div className="mb-4 p-3 bg-emerald-600 text-white rounded-lg shadow-md flex items-center justify-between text-sm font-medium animate-fade-in">
              <div className="flex items-center gap-2">
                <CheckCircle2 className="w-5 h-5" />
                <span>{saveToast}</span>
              </div>
            </div>
          )}

          {/* PAGE: REBOOTING / OTA SUCCESS */}
          {otaState === 'rebooting' ? (
            <div className="max-w-md mx-auto my-12 bg-white p-8 rounded-lg shadow-md text-center">
              <h2 className="text-2xl font-bold text-slate-800 mb-2">
                {lang === 'en' ? '✅ Update Successful!' : '✅ Оновлення успішне!'}
              </h2>
              <p className="text-slate-600 mb-6">
                {lang === 'en' 
                  ? <>Device is rebooting... Returning to dashboard in <b className="text-emerald-600 text-xl">{otaTimer}</b> sec.</>
                  : <>Пристрій перезавантажується... Повернення на головну через <b className="text-emerald-600 text-xl">{otaTimer}</b> сек.</>}
              </p>
              <div className="w-full bg-slate-200 h-2 rounded-full overflow-hidden">
                <div 
                  className="bg-emerald-500 h-full transition-all duration-1000"
                  style={{ width: `${((10 - otaTimer) / 10) * 100}%` }}
                ></div>
              </div>
            </div>
          ) : !bmsData.connected ? (
            /* PAGE: DISCONNECTED STATE */
            <div className="max-w-xl mx-auto my-10 bg-white p-8 rounded-lg shadow-md text-center">
              <h2 className="text-2xl font-bold text-slate-800 mb-2">
                {lang === 'en' ? '⚡️ BMS Status: DISCONNECTED' : '⚡️ Статус BMS: ВІДКЛЮЧЕНО'}
              </h2>
              <p className="text-slate-500 font-mono text-sm mb-6">IP: 192.168.1.185</p>

              <div className="flex flex-wrap justify-center gap-3">
                <button
                  onClick={() => {
                    if (window.confirm(lang === 'en' ? 'Reboot ESP32?' : 'Перезавантажити ESP32?')) {
                      setPage('reboot');
                      setTimeout(() => setPage('dashboard'), 2000);
                    }
                  }}
                  className="px-4 py-2.5 bg-rose-600 text-white rounded font-medium text-sm hover:bg-rose-700 transition"
                >
                  🔄 REBOOT
                </button>
                <button
                  onClick={() => setPage('update')}
                  className="px-4 py-2.5 bg-slate-600 text-white rounded font-medium text-sm hover:bg-slate-700 transition"
                >
                  📥 OTA UPDATE
                </button>
                <button
                  onClick={() => setPage('scan')}
                  className="px-4 py-2.5 bg-blue-600 text-white rounded font-medium text-sm hover:bg-blue-700 transition"
                >
                  {lang === 'en' ? '🔍 SCAN & SELECT BMS' : '🔍 СКАНУВАТИ ТА ВИБРАТИ BMS'}
                </button>
              </div>
            </div>
          ) : page === 'dashboard' ? (
            /* PAGE: DASHBOARD (MAIN TELEMETRY) */
            <div className="max-w-2xl mx-auto bg-white p-5 rounded-lg shadow-sm border border-slate-200">
              <div className="flex flex-wrap items-center justify-between border-b pb-3 mb-4 gap-2">
                <h2 className="text-xl font-bold text-slate-800 flex items-center gap-2">
                  <span>⚡️</span>
                  <span>{lang === 'en' ? 'Live Jikong BMS Telemetry' : 'Живі дані Jikong BMS'}</span>
                </h2>
                <div className="text-xs bg-emerald-100 text-emerald-800 px-2.5 py-1 rounded-full font-medium">
                  {lang === 'en' ? 'BLE Active' : 'BLE Підключено'}
                </div>
              </div>

              {/* Navigation Menu Buttons */}
              <div className="flex flex-wrap justify-center gap-2 mb-5">
                <button
                  onClick={() => setPage('update')}
                  className="px-3 py-1.5 bg-emerald-600 text-white rounded text-sm hover:bg-emerald-700 transition font-medium"
                >
                  📥 OTA UPDATE
                </button>
                <button
                  onClick={() => setPage('settings')}
                  className="px-3 py-1.5 bg-blue-600 text-white rounded text-sm hover:bg-blue-700 transition font-medium"
                >
                  ⚙️ {lang === 'en' ? 'Settings' : 'Налаштування'}
                </button>
                <button
                  onClick={() => setPage('info')}
                  className="px-3 py-1.5 bg-cyan-600 text-white rounded text-sm hover:bg-cyan-700 transition font-medium"
                >
                  ℹ️ {lang === 'en' ? 'Info' : 'Інфо'}
                </button>
                <button
                  onClick={() => setBmsData(d => ({ ...d, connected: false }))}
                  className="px-3 py-1.5 bg-rose-600 text-white rounded text-sm hover:bg-rose-700 transition font-medium"
                >
                  ❌ {lang === 'en' ? 'Disconnect BLE' : 'Вимкнути BLE'}
                </button>
              </div>

              {/* Main Parameter Table */}
              <div className="overflow-x-auto mb-6">
                <table className="w-full border-collapse border border-slate-200 text-sm">
                  <thead>
                    <tr className="bg-blue-600 text-white">
                      <th className="border border-blue-500 p-2 text-left">
                        {lang === 'en' ? 'Parameter' : 'Параметр'}
                      </th>
                      <th className="border border-blue-500 p-2 text-left">
                        {lang === 'en' ? 'Value' : 'Значення'}
                      </th>
                      <th className="border border-blue-500 p-2 text-left">
                        {lang === 'en' ? 'Unit' : 'Од.'}
                      </th>
                    </tr>
                  </thead>
                  <tbody className="divide-y divide-slate-200">
                    <tr className="hover:bg-slate-50">
                      <td className="p-2 font-medium">{lang === 'en' ? 'Battery Voltage' : 'Напруга батареї'}</td>
                      <td className="p-2 font-mono font-bold text-slate-900">{bmsData.batteryVoltage.toFixed(2)}</td>
                      <td className="p-2 text-slate-500">V</td>
                    </tr>
                    <tr className="hover:bg-slate-50">
                      <td className="p-2 font-medium">{lang === 'en' ? 'Charge / Discharge Current' : 'Струм заряду/розряду'}</td>
                      <td className={`p-2 font-mono font-bold ${bmsData.chargeCurrent > 0 ? 'text-emerald-600' : 'text-slate-700'}`}>
                        {bmsData.chargeCurrent.toFixed(2)}
                      </td>
                      <td className="p-2 text-slate-500">A</td>
                    </tr>
                    <tr className="hover:bg-slate-50">
                      <td className="p-2 font-medium">{lang === 'en' ? 'Power' : 'Потужність'}</td>
                      <td className="p-2 font-mono font-bold text-slate-900">{bmsData.power.toFixed(1)}</td>
                      <td className="p-2 text-slate-500">W</td>
                    </tr>
                    <tr className="hover:bg-slate-50">
                      <td className="p-2 font-medium">{lang === 'en' ? 'State of Charge (SOC)' : 'Залишок заряду (SOC)'}</td>
                      <td className="p-2 font-mono font-bold text-emerald-600">{bmsData.soc}%</td>
                      <td className="p-2 text-slate-500">%</td>
                    </tr>
                    <tr className="hover:bg-slate-50">
                      <td className="p-2 font-medium">{lang === 'en' ? 'Cell Delta' : 'Дельта комірок'}</td>
                      <td className="p-2 font-mono font-bold text-blue-600">{bmsData.deltaCellVoltage.toFixed(3)}</td>
                      <td className="p-2 text-slate-500">V</td>
                    </tr>
                    <tr className="hover:bg-slate-50">
                      <td className="p-2 font-medium">{lang === 'en' ? 'Temperature T1 / T2' : 'Температура T1 / T2'}</td>
                      <td className="p-2 font-mono">{bmsData.temp1.toFixed(1)} / {bmsData.temp2.toFixed(1)}</td>
                      <td className="p-2 text-slate-500">°C</td>
                    </tr>
                    <tr className="hover:bg-slate-50">
                      <td className="p-2 font-medium">{lang === 'en' ? 'MOSFET Temperature' : 'Температура MOS'}</td>
                      <td className="p-2 font-mono">{bmsData.tempMos.toFixed(1)}</td>
                      <td className="p-2 text-slate-500">°C</td>
                    </tr>
                    
                    {/* Switch: Charge */}
                    <tr className="hover:bg-slate-50">
                      <td className="p-2 font-medium">{lang === 'en' ? 'Charge Switch' : 'Заряд'}</td>
                      <td className="p-2">
                        {bmsData.chargeEnabled ? (
                          <span className="text-emerald-600 font-semibold">{lang === 'en' ? '🟢 ON' : '🟢 УВІМК'}</span>
                        ) : (
                          <span className="text-rose-600 font-semibold">{lang === 'en' ? '🔴 OFF' : '🔴 ВИМК'}</span>
                        )}
                      </td>
                      <td className="p-2">
                        <button
                          onClick={toggleCharge}
                          className={`px-3 py-1 rounded text-xs font-semibold text-white transition ${
                            bmsData.chargeEnabled ? 'bg-rose-600 hover:bg-rose-700' : 'bg-emerald-600 hover:bg-emerald-700'
                          }`}
                        >
                          {bmsData.chargeEnabled 
                            ? (lang === 'en' ? 'TURN OFF' : 'ВИМК') 
                            : (lang === 'en' ? 'TURN ON' : 'УВІМК')}
                        </button>
                      </td>
                    </tr>

                    {/* Switch: Discharge */}
                    <tr className="hover:bg-slate-50">
                      <td className="p-2 font-medium">{lang === 'en' ? 'Discharge Switch' : 'Розряд'}</td>
                      <td className="p-2">
                        {bmsData.dischargeEnabled ? (
                          <span className="text-emerald-600 font-semibold">{lang === 'en' ? '🟢 ON' : '🟢 УВІМК'}</span>
                        ) : (
                          <span className="text-rose-600 font-semibold">{lang === 'en' ? '🔴 OFF' : '🔴 ВИМК'}</span>
                        )}
                      </td>
                      <td className="p-2">
                        <button
                          onClick={toggleDischarge}
                          className={`px-3 py-1 rounded text-xs font-semibold text-white transition ${
                            bmsData.dischargeEnabled ? 'bg-rose-600 hover:bg-rose-700' : 'bg-emerald-600 hover:bg-emerald-700'
                          }`}
                        >
                          {bmsData.dischargeEnabled 
                            ? (lang === 'en' ? 'TURN OFF' : 'ВИМК') 
                            : (lang === 'en' ? 'TURN ON' : 'УВІМК')}
                        </button>
                      </td>
                    </tr>

                    {/* Switch: Balance */}
                    <tr className="hover:bg-slate-50">
                      <td className="p-2 font-medium">{lang === 'en' ? 'Balance Switch' : 'Баланс'}</td>
                      <td className="p-2" colSpan={2}>
                        <div className="flex items-center gap-3">
                          {bmsData.balanceEnabled ? (
                            <span className="text-emerald-600 font-semibold flex items-center gap-1.5">
                              <span>🟢</span>
                              <span>
                                {lang === 'en' 
                                  ? `Active (${bmsData.balanceCurrent.toFixed(3)} A)` 
                                  : `Активний (${bmsData.balanceCurrent.toFixed(3)} A)`}
                              </span>
                            </span>
                          ) : (
                            <span className="text-slate-500 font-medium">
                              {lang === 'en' ? '⚪ Inactive' : '⚪ Ні'}
                            </span>
                          )}
                          <button
                            onClick={toggleBalance}
                            className={`px-3 py-1 rounded text-xs font-semibold text-white transition ${
                              bmsData.balanceEnabled ? 'bg-rose-600 hover:bg-rose-700' : 'bg-emerald-600 hover:bg-emerald-700'
                            }`}
                          >
                            {bmsData.balanceEnabled 
                              ? (lang === 'en' ? 'TURN OFF' : 'ВИМК') 
                              : (lang === 'en' ? 'TURN ON' : 'УВІМК')}
                          </button>
                        </div>
                      </td>
                    </tr>
                  </tbody>
                </table>
              </div>

              {/* Cell Voltages Table */}
              <h2 className="text-lg font-bold text-slate-800 mb-2 flex items-center gap-2">
                <span>🔬</span>
                <span>{lang === 'en' ? 'Cell Voltages' : 'Напруги комірок'}</span>
              </h2>
              <div className="overflow-x-auto max-h-72 border border-slate-200 rounded">
                <table className="w-full border-collapse text-sm">
                  <thead className="sticky top-0 bg-blue-600 text-white">
                    <tr>
                      <th className="p-2 text-left">{lang === 'en' ? 'Cell' : 'Комірка'}</th>
                      <th className="p-2 text-left">{lang === 'en' ? 'Voltage (V)' : 'Напруга (V)'}</th>
                      <th className="p-2 text-left">{lang === 'en' ? 'Resistance (Ω)' : 'Опір (Ом)'}</th>
                    </tr>
                  </thead>
                  <tbody className="divide-y divide-slate-200">
                    {bmsData.cells.map(cell => (
                      <tr 
                        key={cell.index} 
                        className={`hover:bg-slate-50 ${cell.isBalancing ? 'bg-emerald-50/70' : ''}`}
                      >
                        <td className="p-2 font-medium flex items-center gap-2">
                          <span>{cell.index}</span>
                          {cell.isBalancing && (
                            <span className="px-1.5 py-0.2 bg-emerald-200 text-emerald-800 text-[10px] rounded font-bold uppercase">
                              {lang === 'en' ? 'Balancing' : 'Баланс'}
                            </span>
                          )}
                        </td>
                        <td className="p-2 font-mono font-semibold text-slate-900">{cell.voltage.toFixed(3)}</td>
                        <td className="p-2 font-mono text-slate-500">{cell.resistance.toFixed(3)}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </div>
          ) : page === 'settings' ? (
            /* PAGE: SETTINGS */
            <div className="max-w-2xl mx-auto bg-white p-5 rounded-lg shadow-sm border border-slate-200">
              <div className="flex items-center justify-between border-b pb-3 mb-4">
                <h2 className="text-xl font-bold text-slate-800 flex items-center gap-2">
                  <span>⚙️</span>
                  <span>{lang === 'en' ? 'Jikong BMS Settings' : 'Налаштування Jikong BMS'}</span>
                </h2>
                <button
                  onClick={() => setPage('dashboard')}
                  className="text-blue-600 hover:underline text-sm font-medium"
                >
                  {lang === 'en' ? '⬅️ Back to Dashboard' : '⬅️ На головну'}
                </button>
              </div>

              <form onSubmit={handleSettingsSubmit} className="space-y-4">
                <table className="w-full border-collapse border border-slate-200 text-sm">
                  <thead>
                    <tr className="bg-blue-600 text-white">
                      <th className="p-2 text-left">{lang === 'en' ? 'Parameter' : 'Параметр'}</th>
                      <th className="p-2 text-left">{lang === 'en' ? 'Current' : 'Поточне'}</th>
                      <th className="p-2 text-left">{lang === 'en' ? 'New' : 'Нове'}</th>
                    </tr>
                  </thead>
                  <tbody>
                    {/* Battery Parameters */}
                    <tr className="bg-emerald-600 text-white font-semibold">
                      <th colSpan={3} className="p-2 text-left">
                        {lang === 'en' ? 'Battery Parameters' : 'Параметри батареї'}
                      </th>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Nominal Capacity (Ah)' : 'Номінальна ємність (Ah)'}</td>
                      <td className="p-2 font-mono">{settings.totalBatteryCapacity.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.totalBatteryCapacity}
                          onChange={e => setSettings({ ...settings, totalBatteryCapacity: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Cell Count' : 'Кількість комірок'}</td>
                      <td className="p-2 font-mono">{settings.cellCount}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="1"
                          value={settings.cellCount}
                          onChange={e => setSettings({ ...settings, cellCount: parseInt(e.target.value) || 16 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'BMS Mode' : 'Режим BMS'}</td>
                      <td className="p-2 font-mono">{settings.bmsMode}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="1"
                          value={settings.bmsMode}
                          onChange={e => setSettings({ ...settings, bmsMode: parseInt(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>

                    {/* Cell Voltages */}
                    <tr className="bg-emerald-600 text-white font-semibold">
                      <th colSpan={3} className="p-2 text-left">
                        {lang === 'en' ? 'Cell Voltages (V)' : 'Напруги комірок (V)'}
                      </th>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Overvoltage Protection (OVP)' : 'Захист від перенапруги (ОВР)'}</td>
                      <td className="p-2 font-mono">{settings.cellOvervoltageProtection.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.cellOvervoltageProtection}
                          onChange={e => setSettings({ ...settings, cellOvervoltageProtection: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Undervoltage Protection (UVP)' : 'Захист від низької напруги (УВР)'}</td>
                      <td className="p-2 font-mono">{settings.cellUndervoltageProtection.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.cellUndervoltageProtection}
                          onChange={e => setSettings({ ...settings, cellUndervoltageProtection: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'OVP Recovery Voltage' : 'Відновлення ОВР'}</td>
                      <td className="p-2 font-mono">{settings.cellOvervoltageRecovery.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.cellOvervoltageRecovery}
                          onChange={e => setSettings({ ...settings, cellOvervoltageRecovery: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'UVP Recovery Voltage' : 'Відновлення УВР'}</td>
                      <td className="p-2 font-mono">{settings.cellUndervoltageRecovery.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.cellUndervoltageRecovery}
                          onChange={e => setSettings({ ...settings, cellUndervoltageRecovery: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Power-Off Voltage' : 'Напруга вимкнення (Power Off)'}</td>
                      <td className="p-2 font-mono">{settings.powerOffVoltage.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.powerOffVoltage}
                          onChange={e => setSettings({ ...settings, powerOffVoltage: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>

                    {/* Charging Voltages */}
                    <tr className="bg-emerald-600 text-white font-semibold">
                      <th colSpan={3} className="p-2 text-left">
                        {lang === 'en' ? 'Charging / Nominal Voltages (V)' : 'Зарядні / Номінальні напруги (V)'}
                      </th>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Full Charge Voltage' : 'Повна напруга заряду'}</td>
                      <td className="p-2 font-mono">{settings.fullChargeVoltage.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.fullChargeVoltage}
                          onChange={e => setSettings({ ...settings, fullChargeVoltage: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Nominal Voltage' : 'Номінальна напруга'}</td>
                      <td className="p-2 font-mono">{settings.nominalVoltage.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.nominalVoltage}
                          onChange={e => setSettings({ ...settings, nominalVoltage: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Float Charge Voltage' : 'Підтримуюча напруга заряду'}</td>
                      <td className="p-2 font-mono">{settings.floatChargeVoltage.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.floatChargeVoltage}
                          onChange={e => setSettings({ ...settings, floatChargeVoltage: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>

                    {/* Balancing */}
                    <tr className="bg-emerald-600 text-white font-semibold">
                      <th colSpan={3} className="p-2 text-left">
                        {lang === 'en' ? 'Balancing' : 'Балансування'}
                      </th>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Balance Trigger Voltage (V)' : 'Напруга запуску балансування (V)'}</td>
                      <td className="p-2 font-mono">{settings.balanceTriggerVoltage.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.balanceTriggerVoltage}
                          onChange={e => setSettings({ ...settings, balanceTriggerVoltage: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Max Balance Current (A)' : 'Макс. струм балансування (A)'}</td>
                      <td className="p-2 font-mono">{settings.maxBalanceCurrent.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.maxBalanceCurrent}
                          onChange={e => setSettings({ ...settings, maxBalanceCurrent: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>

                    {/* Current Limits */}
                    <tr className="bg-emerald-600 text-white font-semibold">
                      <th colSpan={3} className="p-2 text-left">
                        {lang === 'en' ? 'Current Limits (A)' : 'СТРУМИ (А)'}
                      </th>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Max Charge Current (A)' : 'Макс. струм заряду (A)'}</td>
                      <td className="p-2 font-mono">{settings.maxChargeCurrent.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.maxChargeCurrent}
                          onChange={e => setSettings({ ...settings, maxChargeCurrent: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                    <tr className="border-b">
                      <td className="p-2">{lang === 'en' ? 'Max Discharge Current (A)' : 'Макс. струм розряду (A)'}</td>
                      <td className="p-2 font-mono">{settings.maxDischargeCurrent.toFixed(3)}</td>
                      <td className="p-2">
                        <input
                          type="number"
                          step="0.001"
                          value={settings.maxDischargeCurrent}
                          onChange={e => setSettings({ ...settings, maxDischargeCurrent: parseFloat(e.target.value) || 0 })}
                          className="w-full p-1 border rounded font-mono text-sm"
                        />
                      </td>
                    </tr>
                  </tbody>
                </table>

                <button
                  type="submit"
                  className="w-full py-3 bg-blue-600 text-white rounded font-bold text-base hover:bg-blue-700 transition shadow"
                >
                  {lang === 'en' ? '💾 SAVE ALL SETTINGS' : '💾 ЗБЕРЕГТИ ВСІ НАЛАШТУВАННЯ'}
                </button>
              </form>
            </div>
          ) : page === 'info' ? (
            /* PAGE: DEVICE INFO */
            <div className="max-w-md mx-auto bg-white p-6 rounded-lg shadow-sm border border-slate-200">
              <h2 className="text-xl font-bold text-slate-800 mb-4 flex items-center gap-2">
                <span>ℹ️</span>
                <span>{lang === 'en' ? 'Device Information' : 'Інформація'}</span>
              </h2>

              <table className="w-full border-collapse text-sm mb-6">
                <thead>
                  <tr className="bg-slate-100 text-slate-700 border-b">
                    <th className="p-2 text-left">{lang === 'en' ? 'Parameter' : 'Параметр'}</th>
                    <th className="p-2 text-left">{lang === 'en' ? 'Value' : 'Значення'}</th>
                  </tr>
                </thead>
                <tbody className="divide-y divide-slate-100">
                  <tr>
                    <td className="p-2 font-medium">{lang === 'en' ? 'Device Name' : 'Пристрій'}</td>
                    <td className="p-2 font-mono font-semibold">{deviceInfo.deviceName}</td>
                  </tr>
                  <tr>
                    <td className="p-2 font-medium">{lang === 'en' ? 'Serial Number' : 'Серійний №'}</td>
                    <td className="p-2 font-mono">{deviceInfo.serialNumber}</td>
                  </tr>
                  <tr>
                    <td className="p-2 font-medium">{lang === 'en' ? 'Hardware / Software Version' : 'Версія HW/SW'}</td>
                    <td className="p-2 font-mono">{deviceInfo.hardwareVersion} / {deviceInfo.softwareVersion}</td>
                  </tr>
                  <tr>
                    <td className="p-2 font-medium">{lang === 'en' ? 'Firmware Build Time' : 'Компіляція'}</td>
                    <td className="p-2 font-mono text-emerald-600 font-bold">{deviceInfo.buildTime}</td>
                  </tr>
                  <tr>
                    <td className="p-2 font-medium">Vendor ID</td>
                    <td className="p-2 font-mono">{deviceInfo.vendorId}</td>
                  </tr>
                  <tr>
                    <td className="p-2 font-medium">{lang === 'en' ? 'Manufacturing Date' : 'Дата виготовлення'}</td>
                    <td className="p-2 font-mono">{deviceInfo.manufacturingDate}</td>
                  </tr>
                  <tr>
                    <td className="p-2 font-medium">Device Passcode</td>
                    <td className="p-2 font-mono">********</td>
                  </tr>
                  <tr>
                    <td className="p-2 font-medium">{lang === 'en' ? 'User Info' : 'Користувач'}</td>
                    <td className="p-2">{deviceInfo.userData}</td>
                  </tr>
                </tbody>
              </table>

              <div className="text-center">
                <button
                  onClick={() => setPage('dashboard')}
                  className="px-4 py-2 bg-blue-600 text-white rounded font-medium text-sm hover:bg-blue-700 transition"
                >
                  {lang === 'en' ? '◀️ DASHBOARD' : '◀️ ГОЛОВНА'}
                </button>
              </div>
            </div>
          ) : page === 'scan' ? (
            /* PAGE: BLE SCANNER */
            <div className="max-w-xl mx-auto bg-white p-6 rounded-lg shadow-sm border border-slate-200">
              <h2 className="text-xl font-bold text-slate-800 mb-4 flex items-center gap-2">
                <span>🔍</span>
                <span>{lang === 'en' ? 'BLE BMS Scanner' : 'Сканування BMS'}</span>
              </h2>

              <table className="w-full border-collapse border border-slate-200 text-sm mb-6">
                <thead>
                  <tr className="bg-blue-600 text-white">
                    <th className="p-2 text-left">{lang === 'en' ? 'Device Name' : "Ім'я пристрою"}</th>
                    <th className="p-2 text-left">MAC</th>
                    <th className="p-2 text-left">{lang === 'en' ? 'Action' : 'Дія'}</th>
                  </tr>
                </thead>
                <tbody className="divide-y divide-slate-200">
                  {scannedDevices.map(dev => (
                    <tr key={dev.address} className="hover:bg-slate-50">
                      <td className="p-2 font-medium flex items-center gap-1.5">
                        <Bluetooth className="w-3.5 h-3.5 text-blue-600" />
                        <span>{dev.name}</span>
                      </td>
                      <td className="p-2 font-mono text-xs">{dev.address}</td>
                      <td className="p-2">
                        <button
                          onClick={() => {
                            setBmsData(d => ({ ...d, connected: true }));
                            setPage('dashboard');
                          }}
                          className="px-3 py-1 bg-emerald-600 text-white rounded text-xs font-semibold hover:bg-emerald-700 transition"
                        >
                          {lang === 'en' ? 'Connect' : 'Підключитись'}
                        </button>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>

              <div className="text-center">
                <button
                  onClick={() => setPage('dashboard')}
                  className="text-blue-600 hover:underline text-sm font-medium"
                >
                  {lang === 'en' ? '◀️ Back to Dashboard' : '◀️ На головну'}
                </button>
              </div>
            </div>
          ) : page === 'update' ? (
            /* PAGE: OTA UPDATE */
            <div className="max-w-md mx-auto bg-white p-8 rounded-lg shadow-sm border border-slate-200 text-center">
              <h1 className="text-2xl font-bold text-slate-800 mb-2">OTA Update</h1>
              <p className="text-slate-600 text-sm mb-6">
                {lang === 'en' ? 'Select firmware binary file (.bin):' : 'Оберіть файл прошивки (.bin):'}
              </p>

              <div className="border-2 border-dashed border-slate-300 rounded-lg p-6 mb-6 bg-slate-50">
                <UploadCloud className="w-10 h-10 text-slate-400 mx-auto mb-2" />
                <div className="text-xs text-slate-500 mb-3 font-mono">
                  esp32_jk_bms_english.bin (1,048,576 bytes)
                </div>
                
                {otaState === 'uploading' ? (
                  <div className="space-y-2">
                    <div className="text-xs font-semibold text-blue-600">
                      {lang === 'en' ? `Flashing firmware: ${otaProgress}%` : `Запис прошивки: ${otaProgress}%`}
                    </div>
                    <div className="w-full bg-slate-200 h-2 rounded-full overflow-hidden">
                      <div 
                        className="bg-blue-600 h-full transition-all duration-300"
                        style={{ width: `${otaProgress}%` }}
                      ></div>
                    </div>
                  </div>
                ) : (
                  <button
                    onClick={handleSimulatedOta}
                    className="px-6 py-2 bg-blue-600 text-white font-medium text-sm rounded hover:bg-blue-700 transition shadow"
                  >
                    {lang === 'en' ? 'Upload & Update' : 'Оновити'}
                  </button>
                )}
              </div>

              <button
                onClick={() => setPage('dashboard')}
                className="text-blue-600 hover:underline text-sm font-medium"
              >
                {lang === 'en' ? '◀️ Back to Dashboard' : '◀️ На головну'}
              </button>
            </div>
          ) : null}

        </div>
      </div>
    </div>
  );
};
