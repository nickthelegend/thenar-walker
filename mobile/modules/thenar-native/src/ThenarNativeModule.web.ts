import { registerWebModule, NativeModule } from 'expo';

import { GamepadState, ThenarNativeModuleEvents } from './ThenarNative.types';

// Web build (laptop browser): same events from the browser Gamepad API. Wi-Fi binding is not needed there.
class ThenarNativeModule extends NativeModule<ThenarNativeModuleEvents> {
  private timer: ReturnType<typeof setInterval> | null = null;
  private last = '';

  startGamepad() {
    if (this.timer || typeof navigator === 'undefined' || !navigator.getGamepads) return;
    this.timer = setInterval(() => {
      const g = [...navigator.getGamepads()].find((p) => p && p.connected);
      const s: GamepadState = g
        ? {
            connected: true,
            name: g.id,
            axes: [g.axes[0] ?? 0, g.axes[1] ?? 0, g.axes[2] ?? 0, g.axes[3] ?? 0, g.buttons[6]?.value ?? 0, g.buttons[7]?.value ?? 0],
            buttons: g.buttons.reduce((m, b, i) => (b.pressed && i < 31 ? m | (1 << i) : m), 0),
          }
        : { connected: false, name: '', axes: [0, 0, 0, 0, 0, 0], buttons: 0 };
      const key = JSON.stringify(s);
      if (key !== this.last) {
        this.last = key;
        this.emit('onGamepad', s);
      }
    }, 16);
  }

  bindToWifi(_ssid: string, _password: string) {}

  unbindWifi() {}
}

export default registerWebModule(ThenarNativeModule, 'ThenarNativeModule');
