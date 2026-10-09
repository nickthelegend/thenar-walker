import Slider from '@react-native-community/slider';
import { useKeepAwake } from 'expo-keep-awake';
import * as SecureStore from 'expo-secure-store';
import { StatusBar } from 'expo-status-bar';
import { useEffect, useMemo, useRef, useState, useSyncExternalStore } from 'react';
import {
  Alert, LayoutChangeEvent, PanResponder, Platform, Pressable, ScrollView, StyleSheet, Text, TextInput, View, useWindowDimensions,
} from 'react-native';
import { SafeAreaProvider, SafeAreaView } from 'react-native-safe-area-context';

import ThenarNative, { GamepadState } from './modules/thenar-native';
import { NO_PAD, PadMapper } from './src/pad';
import { GRIPPER, JOINTS, RobotLink, SPEEDS, Snapshot } from './src/robot';

const C = { bg: '#0f141a', card: '#18202a', line: '#2a3542', fg: '#e8edf2', dim: '#93a3b5', acc: '#f08a3c', ok: '#3ccf7a', bad: '#ff5a5a', warn: '#ffb36b' };
const DEFAULT_HOST = '192.168.4.1';
const DEFAULT_SSID = 'ThenarWalker_TF';

// settings: SecureStore on the phone, localStorage on the web build
const store = {
  async get(k: string) {
    try {
      return Platform.OS === 'web' ? globalThis.localStorage?.getItem(k) ?? null : await SecureStore.getItemAsync(k);
    } catch {
      return null;
    }
  },
  async set(k: string, v: string) {
    try {
      if (Platform.OS === 'web') globalThis.localStorage?.setItem(k, v);
      else await SecureStore.setItemAsync(k, v);
    } catch {}
  },
};

export default function App() {
  return (
    <SafeAreaProvider>
      <Remote />
    </SafeAreaProvider>
  );
}

function Remote() {
  useKeepAwake();
  const link = useMemo(() => new RobotLink(DEFAULT_HOST), []);
  const mapper = useMemo(() => new PadMapper(link), [link]);
  const s = useSyncExternalStore(link.subscribe, link.getSnapshot);
  const [pad, setPad] = useState<GamepadState>(NO_PAD);
  const [wifi, setWifi] = useState('');
  const [showSettings, setShowSettings] = useState(false);
  const [host, setHost] = useState(DEFAULT_HOST);
  const [ssid, setSsid] = useState(DEFAULT_SSID);
  const [password, setPassword] = useState('');
  const { width, height } = useWindowDimensions();
  const wide = width > height && width >= 700;

  useEffect(() => {
    link.onTick = (dt) => mapper.step(dt);
    link.start(100);
    const subs = [
      ThenarNative.addListener('onGamepad', (st) => { mapper.update(st); setPad(st); }),
      ThenarNative.addListener('onWifi', (e) => setWifi(e.bound ? 'robot Wi-Fi in use' : e.reason === 'unavailable' ? 'Wi-Fi not found' : '')),
    ];
    ThenarNative.startGamepad();
    (async () => {
      const h = (await store.get('host')) || DEFAULT_HOST, id = (await store.get('ssid')) || DEFAULT_SSID, pw = (await store.get('password')) || '';
      setHost(h); setSsid(id); setPassword(pw);
      link.setHost(h);
      ThenarNative.bindToWifi('', ''); // use the Wi-Fi the phone is on even though it has no internet
    })();
    return () => { subs.forEach((x) => x.remove()); link.stopLoop(); ThenarNative.unbindWifi(); };
  }, [link, mapper]);

  const saveSettings = async (join: boolean) => {
    await store.set('host', host.trim());
    await store.set('ssid', ssid.trim());
    await store.set('password', password);
    link.setHost(host);
    ThenarNative.bindToWifi(join ? ssid.trim() : '', join ? password : '');
    if (join) setWifi('joining ' + ssid.trim() + '…');
    setShowSettings(false);
  };

  const drive = (
    <Card title="Drive">
      <Joystick onMove={(x, y) => (link.touchDrive = { thr: y, trn: -x })} />
      <View style={st.row}>
        <Text style={st.dim}>Speed</Text>
        <View style={st.seg}>
          {SPEEDS.map((n, i) => (
            <Btn key={n} label={n} sel={s.sp === i} onPress={() => link.setSpeed(i)} style={st.segBtn} />
          ))}
        </View>
      </View>
      <Text style={st.note}>Let go = wheels stop. If the robot hears nothing for 0.5 s it stops the wheels itself.</Text>
    </Card>
  );

  const arm = (
    <Card title="Arm">
      <View style={st.row}>
        <Btn label="All to 0°" onPress={() => link.zero()} />
        <Btn label="All joints ON" onPress={() => link.allOn()} />
        <Btn label="Limp all" onPress={() => link.limp()} />
      </View>
      <View style={[st.row, { marginTop: 8 }]}>
        <Text style={st.dim}>Gripper</Text>
        <Btn label="Open" onPress={() => link.gripper(true)} disabled={!link.usable(GRIPPER)} />
        <Btn label="Close" onPress={() => link.gripper(false)} disabled={!link.usable(GRIPPER)} />
      </View>
      {JOINTS.map((j, i) => <JointRow key={j.id} i={i} s={s} link={link} />)}
      {!!calNote(s) && <Text style={[st.note, { color: C.warn }]}>{calNote(s)}</Text>}
      <Text style={st.note}>A joint is limp until it is ON — then it goes straight to its target, so hold the arm near there first.</Text>
    </Card>
  );

  const gamepad = (
    <Card title="Gamepad">
      <Text style={st.note}>
        {pad.connected ? `On this phone: ${pad.name}${mapper.fine ? '  (precision arm)' : ''}` : 'Pair a Bluetooth controller with this phone and press any button.'}
        {s.robotPad ? `\nOn the robot: ${s.robotPad}` : ''}
      </Text>
      {[
        ['Left stick', 'drive'], ['Right stick', 'J1 pan · J2 shoulder'], ['D-pad ↑ ↓', 'J3 elbow'], ['D-pad ← →', 'J5 wrist roll'],
        ['Y / A', 'J4 wrist up / down'], ['L2 / R2', 'gripper open / close'], ['L1 / R1', 'speed down / up'], ['B', 'STOP'],
        ['X', 'precision arm'], ['hold START 1 s', 'arm ON'], ['hold SELECT 1 s', 'arm limp'],
      ].map(([k, v]) => (
        <View key={k} style={st.mapRow}><Text style={st.mapKey}>{k}</Text><Text style={st.mapVal}>{v}</Text></View>
      ))}
      {!!s.cfg?.bt && (
        <Btn label="Forget robot gamepad" style={{ marginTop: 10, alignSelf: 'flex-start' }}
          onPress={() => confirmAsk('Forget the robot gamepad?', 'The next controller that connects to the robot is accepted.', () => link.forgetPad())} />
      )}
    </Card>
  );

  return (
    <SafeAreaView style={st.screen} edges={['top', 'left', 'right']}>
      <StatusBar style="light" />
      <View style={st.header}>
        <Text style={st.title} numberOfLines={1}>THENAR</Text>
        <Pill text={pad.connected ? 'pad' : s.robotPad ? 'robot pad' : 'no pad'} tone={pad.connected || s.robotPad ? 'ok' : undefined} />
        <Pill text={s.link === 'ok' ? 'connected' : s.link === 'lost' ? 'NO LINK' : 'connecting'} tone={s.link === 'ok' ? 'ok' : s.link === 'lost' ? 'bad' : undefined} />
        <Pill text={s.bat > 1 ? `${s.bat.toFixed(1)} V` : '-- V'} />
        <Pressable onPress={() => setShowSettings((v) => !v)} hitSlop={10} accessibilityRole="button" accessibilityLabel="Connection settings"><Text style={st.gear}>⚙</Text></Pressable>
      </View>
      <ScrollView contentContainerStyle={st.main} keyboardShouldPersistTaps="handled">
        {showSettings && (
          <Card title="Connection">
            <Field label="Robot address" value={host} onChange={setHost} />
            <Field label="Robot Wi-Fi name" value={ssid} onChange={setSsid} />
            <Field label="Wi-Fi password" value={password} onChange={setPassword} secure />
            <View style={st.row}>
              <Btn label="Join robot Wi-Fi" sel onPress={() => saveSettings(true)} />
              <Btn label="Use current Wi-Fi" onPress={() => saveSettings(false)} />
            </View>
            <Text style={st.note}>Join asks Android to connect to the robot (one tap to allow). The password stays on this phone.</Text>
          </Card>
        )}
        {s.link !== 'ok' && !showSettings && (
          <Text style={[st.note, { color: C.warn, textAlign: 'center' }]}>
            Can't reach the robot at {link.host}. Join its Wi-Fi ({ssid}) — tap ⚙.{wifi ? `\n${wifi}` : ''}
          </Text>
        )}
        <Pressable onPress={() => link.stop()} accessibilityRole="button" accessibilityLabel="STOP" style={({ pressed }) => [st.stop, pressed && { opacity: 0.8 }]}>
          <Text style={st.stopText}>STOP</Text>
        </Pressable>
        {wide ? (
          <View style={{ flexDirection: 'row', gap: 12, alignItems: 'flex-start' }}>
            <View style={{ flex: 1, gap: 12 }}>{drive}{gamepad}</View>
            <View style={{ flex: 1.3 }}>{arm}</View>
          </View>
        ) : (
          <>{drive}{arm}{gamepad}</>
        )}
      </ScrollView>
    </SafeAreaView>
  );
}

function calNote(s: Snapshot) {
  if (!s.cfg) return '';
  if (!s.cfg.pca) return 'PCA9685 not found — check SDA/SCL wiring and servo board power.';
  if (!s.cfg.cal) return 'No arm calibration on the robot: open the calibrator on the laptop, press Save to robot, then reboot.';
  if (s.cfg.usable !== 63) return 'Joints marked “not calibrated” stay off — finish them in the calibrator and Save to robot.';
  return '';
}

function confirmAsk(title: string, msg: string, ok: () => void) {
  if (Platform.OS === 'web') { if (globalThis.confirm?.(`${title}\n${msg}`)) ok(); return; }
  Alert.alert(title, msg, [{ text: 'Cancel', style: 'cancel' }, { text: 'Forget', style: 'destructive', onPress: ok }]);
}

function JointRow({ i, s, link }: { i: number; s: Snapshot; link: RobotLink }) {
  const ok = link.usable(i), on = ((s.mask >> i) & 1) === 1, cfg = s.cfg;
  const lo = cfg?.lo[i] ?? -90, hi = cfg?.hi[i] ?? 90;
  return (
    <View style={st.joint}>
      <View style={st.row}>
        <View style={{ flex: 1 }}>
          <Text style={st.fg}><Text style={{ fontWeight: '700' }}>{JOINTS[i].id}</Text> {JOINTS[i].name}</Text>
          <Text style={st.small}>{ok ? `${lo.toFixed(0)}° … ${hi.toFixed(0)}°` : 'not calibrated'}</Text>
        </View>
        <Text style={st.val}>{s.q[i].toFixed(1)}°</Text>
        <Btn label={on ? 'ON' : 'OFF'} onPress={() => link.toggle(i)} disabled={!ok} style={[{ minWidth: 62 }, on && st.onBtn]} textStyle={on ? { color: '#08130c', fontWeight: '700' } : undefined} />
      </View>
      <Slider
        key={`${lo}:${hi}`} // remount when the limits arrive: the slider ignores later min/max changes
        style={{ height: 40 }}
        minimumValue={lo}
        maximumValue={hi}
        step={0.5}
        value={s.q[i] || 1e-6} // the web slider treats 0 as "unset" and jumps to the minimum
        disabled={!ok}
        minimumTrackTintColor={C.acc}
        maximumTrackTintColor={C.line}
        thumbTintColor={C.acc}
        onSlidingStart={() => (link.touching = i)}
        onValueChange={(v) => link.setJoint(i, v)}
        onSlidingComplete={(v) => { link.setJoint(i, v); link.touching = -1; }}
      />
      <View style={st.steps}>
        {[-5, -1, 1, 5].map((d) => (
          <Btn key={d} label={`${d > 0 ? '+' : ''}${d}°`} onPress={() => link.nudge(i, d)} disabled={!ok} style={{ flex: 1 }} />
        ))}
      </View>
    </View>
  );
}

function Joystick({ onMove }: { onMove: (x: number, y: number) => void }) {
  const [size, setSize] = useState(0);
  const [knob, setKnob] = useState({ x: 0, y: 0 });
  const start = useRef({ x: 0, y: 0 });
  const r = size / 2;
  const set = (x: number, y: number) => {
    const m = Math.hypot(x, y);
    if (m > 1) { x /= m; y /= m; }
    setKnob({ x, y });
    onMove(x, y);
  };
  const responder = useMemo(
    () => PanResponder.create({
      onStartShouldSetPanResponder: () => true,
      onMoveShouldSetPanResponder: () => true,
      onPanResponderTerminationRequest: () => false,
      onPanResponderGrant: (e) => {
        start.current = { x: e.nativeEvent.locationX - r, y: e.nativeEvent.locationY - r };
        set(start.current.x / (r * 0.8), -start.current.y / (r * 0.8));
      },
      onPanResponderMove: (_, g) => set((start.current.x + g.dx) / (r * 0.8), -(start.current.y + g.dy) / (r * 0.8)),
      onPanResponderRelease: () => set(0, 0),
      onPanResponderTerminate: () => set(0, 0),
    }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [r],
  );
  const k = size * 0.34;
  return (
    <View style={st.padWrap}>
      <View style={st.pad} onLayout={(e: LayoutChangeEvent) => setSize(e.nativeEvent.layout.width)} {...responder.panHandlers}>
        <View pointerEvents="none" style={[st.knob, { width: k, height: k, borderRadius: k / 2, left: r - k / 2 + knob.x * r * 0.8, top: r - k / 2 - knob.y * r * 0.8 }]} />
      </View>
    </View>
  );
}

function Card({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <View style={st.card}>
      <Text style={st.h2}>{title.toUpperCase()}</Text>
      {children}
    </View>
  );
}

function Pill({ text, tone }: { text: string; tone?: 'ok' | 'bad' }) {
  return (
    <View style={[st.pill, tone === 'ok' && { backgroundColor: C.ok }, tone === 'bad' && { backgroundColor: C.bad }]}>
      <Text style={[st.pillText, tone === 'ok' && { color: '#08130c' }, tone === 'bad' && { color: '#fff' }]}>{text}</Text>
    </View>
  );
}

function Btn({ label, onPress, sel, disabled, style, textStyle }: {
  label: string; onPress: () => void; sel?: boolean; disabled?: boolean; style?: object | (object | false | undefined)[]; textStyle?: object;
}) {
  return (
    <Pressable onPress={onPress} disabled={disabled} accessibilityRole="button" accessibilityLabel={label}
      style={({ pressed }) => [st.btn, sel && st.btnSel, disabled && { opacity: 0.35 }, pressed && { opacity: 0.7 }, style as object]}>
      <Text style={[st.btnText, sel && { color: '#1a0d02', fontWeight: '700' }, textStyle]}>{label}</Text>
    </Pressable>
  );
}

function Field({ label, value, onChange, secure }: { label: string; value: string; onChange: (v: string) => void; secure?: boolean }) {
  return (
    <View style={{ marginBottom: 10 }}>
      <Text style={st.small}>{label}</Text>
      <TextInput value={value} onChangeText={onChange} secureTextEntry={secure} autoCapitalize="none" autoCorrect={false}
        style={st.input} placeholderTextColor={C.dim} />
    </View>
  );
}

const st = StyleSheet.create({
  screen: { flex: 1, backgroundColor: C.bg },
  header: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 14, paddingVertical: 10, borderBottomWidth: 1, borderColor: C.line },
  title: { flex: 1, color: C.fg, fontSize: 16, fontWeight: '800', letterSpacing: 0.5 },
  gear: { color: C.dim, fontSize: 22, paddingLeft: 4 },
  main: { padding: 12, gap: 12, paddingBottom: 40 },
  card: { backgroundColor: C.card, borderWidth: 1, borderColor: C.line, borderRadius: 14, padding: 12 },
  h2: { color: C.dim, fontSize: 12, letterSpacing: 1, marginBottom: 10, fontWeight: '600' },
  row: { flexDirection: 'row', alignItems: 'center', gap: 8, flexWrap: 'wrap' },
  seg: { flexDirection: 'row', flex: 1, gap: 4 },
  segBtn: { flex: 1 },
  fg: { color: C.fg, fontSize: 15 },
  dim: { color: C.dim },
  small: { color: C.dim, fontSize: 12 },
  note: { color: C.dim, fontSize: 12.5, lineHeight: 18, marginTop: 8 },
  pill: { backgroundColor: C.card, borderRadius: 99, paddingHorizontal: 8, paddingVertical: 3 },
  pillText: { color: C.dim, fontSize: 11.5 },
  stop: { backgroundColor: C.bad, borderRadius: 12, paddingVertical: 16, alignItems: 'center' },
  stopText: { color: '#fff', fontSize: 20, fontWeight: '800', letterSpacing: 1 },
  btn: { backgroundColor: '#232d39', borderWidth: 1, borderColor: C.line, borderRadius: 9, paddingVertical: 9, paddingHorizontal: 12, alignItems: 'center' },
  btnSel: { backgroundColor: C.acc, borderColor: C.acc },
  btnText: { color: C.fg, fontSize: 14 },
  onBtn: { backgroundColor: C.ok, borderColor: C.ok },
  joint: { borderTopWidth: 1, borderColor: C.line, paddingTop: 10, marginTop: 10 },
  val: { color: C.fg, fontSize: 18, fontWeight: '600', minWidth: 72, textAlign: 'right', fontVariant: ['tabular-nums'] },
  steps: { flexDirection: 'row', gap: 6 },
  padWrap: { alignItems: 'center', marginVertical: 6 },
  pad: { width: 240, height: 240, borderRadius: 120, backgroundColor: '#141b23', borderWidth: 2, borderColor: C.line },
  knob: { position: 'absolute', backgroundColor: C.acc },
  mapRow: { flexDirection: 'row', borderTopWidth: 1, borderColor: C.line, paddingVertical: 4 },
  mapKey: { color: C.acc, fontWeight: '600', width: 130, fontSize: 13 },
  mapVal: { color: C.fg, flex: 1, fontSize: 13 },
  input: { color: C.fg, borderWidth: 1, borderColor: C.line, borderRadius: 8, paddingHorizontal: 10, paddingVertical: 8, marginTop: 4, backgroundColor: '#11171e' },
});
