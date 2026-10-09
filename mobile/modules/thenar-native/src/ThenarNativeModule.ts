import { NativeModule, requireNativeModule } from 'expo';

import { ThenarNativeModuleEvents } from './ThenarNative.types';

declare class ThenarNativeModule extends NativeModule<ThenarNativeModuleEvents> {
  startGamepad(): void;
  bindToWifi(ssid: string, password: string): void;
  unbindWifi(): void;
}

export default requireNativeModule<ThenarNativeModule>('ThenarNative');
