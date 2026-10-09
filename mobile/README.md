# Thenar Remote (Expo app)

Android app that drives the Thenar Walker: wheels, the 6-joint arm, and a Bluetooth game controller paired to the
phone. It talks to the robot firmware [`firmware/thenar_phone`](../firmware/thenar_phone) over the robot's own Wi-Fi
(`ThenarWalker_TF`, WPA2) at `http://192.168.4.1`.

## What it does that the robot's web page can't
- **Reads the game controller natively.** Any controller Android accepts (Switch-style, Xbox, PS, 8BitDo...) — no
  Chrome flag, no BLE limit. Buttons are consumed, so B doesn't act as Back.
- **Joins and pins the robot Wi-Fi.** Android normally sends traffic over mobile data when the Wi-Fi has no internet.
  The app binds itself to the robot network (⚙ → *Join robot Wi-Fi* asks Android once, Android 10+).
- Keeps the screen on, remembers the Wi-Fi password in the phone's secure storage.

## Controls
Screen: joystick (let go = stop), Slow/Mid/Fast, STOP, a slider + ±1/±5° + ON/OFF per joint, gripper open/close.

| Controller | Does |
|---|---|
| Left stick | drive |
| Right stick | J1 base pan (x), J2 shoulder (y) |
| D-pad ↑↓ / ←→ | J3 elbow / J5 wrist roll |
| Y / A | J4 wrist flex up / down |
| L2 / R2 | gripper open / close |
| L1 / R1 | speed down / up |
| **B** | **STOP** (wheels stay stopped until the stick is centred) |
| X | precision arm (20°/s instead of 60°/s) |
| hold START 1 s | arm ON — every calibrated joint (support the arm: joints jump to their target) |
| hold SELECT 1 s | arm limp |

Safety lives in the robot, not the phone: app silent for 0.5 s → wheels stop and the arm holds; the robot clamps every
angle to the calibrated safe ends; uncalibrated joints can't be switched on; the KCD4 kill switch cuts power.

## Install
Copy `Thenar_Remote.apk` to the phone and open it (allow "install unknown apps" for your file manager once).
Or with USB debugging: `adb install -r Thenar_Remote.apk`.

## Develop
```
npm install
npm test                 # end-to-end: app logic + gamepad mapping against a robot simulator (test/mock_robot.py)
npm run typecheck && npx expo lint
npm run mock             # robot simulator on 127.0.0.1:8793
npx expo start --web     # web build; ⚙ → robot address 127.0.0.1:8793 to drive the simulator
```
Build the APK (needs JDK 17 + Android SDK; the native module means Expo Go can't run it):
```
npx expo prebuild -p android
cd android && gradlew assembleRelease -PreactNativeArchitectures=arm64-v8a
# -> android/app/build/outputs/apk/release/app-release.apk
```

## Layout
- `App.tsx` — the screen
- `src/robot.ts` — robot link (protocol, 10 Hz exchange, merge with the robot's own state)
- `src/pad.ts` — controller layout (same as the firmware's Bluepad32 mapping)
- `modules/thenar-native` — local Expo module (Kotlin): gamepad events + Wi-Fi binding; web fallback uses the
  browser Gamepad API
- `test/` — `mock_robot.py` (firmware rules: slew, clamps, STOP latch, 0.5 s failsafe) and `e2e.test.ts`
