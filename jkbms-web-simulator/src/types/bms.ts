// Types for ESP32 JK-BMS Application

export interface CellData {
  index: number;
  voltage: number;
  resistance: number;
  isBalancing?: boolean;
}

export interface BMSData {
  connected: boolean;
  batteryVoltage: number;
  chargeCurrent: number;
  power: number;
  soc: number;
  deltaCellVoltage: number;
  temp1: number;
  temp2: number;
  tempMos: number;
  balanceCurrent: number;
  chargeEnabled: boolean;
  dischargeEnabled: boolean;
  balanceEnabled: boolean;
  uptimeSeconds: number;
  cycleCount: number;
  capacityRemain: number;
  nominalCapacity: number;
  cells: CellData[];
}

export interface BMSSettings {
  cellUndervoltageProtection: number;
  cellUndervoltageRecovery: number;
  cellOvervoltageProtection: number;
  cellOvervoltageRecovery: number;
  balanceTriggerVoltage: number;
  balanceStartingVoltage: number;
  fullChargeVoltage: number;
  nominalVoltage: number;
  floatChargeVoltage: number;
  balancePrecision: number;
  powerOffVoltage: number;
  maxChargeCurrent: number;
  maxDischargeCurrent: number;
  maxBalanceCurrent: number;
  cellCount: number;
  totalBatteryCapacity: number;
  bmsMode: number;
  chargeOvercurrentProtectionDelay?: number;
  dischargeOvercurrentProtectionDelay?: number;
  shortCircuitProtectionDelay?: number;
}

export interface DeviceInfo {
  vendorId: string;
  hardwareVersion: string;
  softwareVersion: string;
  deviceName: string;
  devicePasscode: string;
  manufacturingDate: string;
  serialNumber: string;
  passcode: string;
  userData: string;
  setupPasscode: string;
  buildTime: string;
}

export interface ScannedDevice {
  name: string;
  address: string;
  rssi: number;
}

export type SimulatorPage = 'dashboard' | 'settings' | 'info' | 'scan' | 'update' | 'reboot';
