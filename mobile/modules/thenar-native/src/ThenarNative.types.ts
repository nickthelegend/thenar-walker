// Gamepad state in the W3C standard-gamepad button order (see ThenarNativeModule.kt).
export type GamepadState = {
  connected: boolean;
  name: string;
  axes: number[]; // [lx, ly, rx, ry, l2, r2]; sticks -1..1 with down = +, triggers 0..1
  buttons: number; // bit i = standard button i pressed
};

export type WifiEvent = { bound: boolean; reason: string };

export type ThenarNativeModuleEvents = {
  onGamepad: (state: GamepadState) => void;
  onWifi: (event: WifiEvent) => void;
};
