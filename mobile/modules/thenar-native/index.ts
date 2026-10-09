// Re-export the native module. On web, it will be resolved to ThenarNativeModule.web.ts
// and on native platforms to ThenarNativeModule.ts
export { default } from './src/ThenarNativeModule';
export * from './src/ThenarNative.types';
