package expo.modules.thenarnative

import android.content.Context
import android.hardware.input.InputManager
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.net.wifi.WifiNetworkSpecifier
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.view.InputDevice
import android.view.KeyEvent
import android.view.MotionEvent
import android.view.Window
import expo.modules.kotlin.modules.Module
import expo.modules.kotlin.modules.ModuleDefinition

// Two things a web page cannot do well on Android:
//  1. Gamepad input. The activity's Window.Callback is wrapped so key + joystick events from a game controller are
//     read here (and consumed, so B does not act as Back). State is sent to JS as "onGamepad" using the W3C
//     standard-gamepad button order: 0 A, 1 B, 2 X, 3 Y, 4 L1, 5 R1, 6 L2, 7 R2, 8 select, 9 start, 10 L3, 11 R3,
//     12-15 d-pad up/down/left/right, 16 home. axes = [lx, ly, rx, ry, l2, r2], sticks -1..1 (down = +), triggers 0..1.
//  2. Talking to the robot's Wi-Fi, which has no internet: Android keeps mobile data as the default route, so the
//     process is bound to the robot network (optionally joining it by SSID/password on Android 10+).
class ThenarNativeModule : Module() {
  private val main = Handler(Looper.getMainLooper())
  private var hookedWindow: Window? = null
  private val axes = FloatArray(6)
  private var keyBits = 0
  private var hatBits = 0
  private var padName = ""
  private var padId = -1
  private var deviceListener: InputManager.InputDeviceListener? = null
  private var netCallback: ConnectivityManager.NetworkCallback? = null

  override fun definition() = ModuleDefinition {
    Name("ThenarNative")
    Events("onGamepad", "onWifi")

    OnCreate { main.post { hook() } }
    OnActivityEntersForeground { main.post { hook() } }
    OnActivityEntersBackground { main.post { clearPad() } }   // never leave a stick "held" while the app is hidden
    OnDestroy {
      main.post { unhook() }
      unbindWifi()
    }

    Function("startGamepad") { main.post { hook() } }

    // ssid/password empty: bind to whatever Wi-Fi the phone is on. Otherwise (Android 10+) ask the system to join it.
    Function("bindToWifi") { ssid: String, password: String -> bindWifi(ssid, password) }
    Function("unbindWifi") { unbindWifi() }
  }

  // ------------------------------------------------------------------ gamepad
  private fun hook() {
    val activity = appContext.currentActivity ?: return
    val window = activity.window ?: return
    if (window === hookedWindow) return
    val inner = window.callback ?: return
    window.callback = object : Window.Callback by inner {
      override fun dispatchKeyEvent(event: KeyEvent): Boolean = onKey(event) || inner.dispatchKeyEvent(event)
      override fun dispatchGenericMotionEvent(event: MotionEvent): Boolean =
        onMotion(event) || inner.dispatchGenericMotionEvent(event)
    }
    hookedWindow = window
    val im = activity.getSystemService(Context.INPUT_SERVICE) as InputManager
    if (deviceListener == null) {
      val l = object : InputManager.InputDeviceListener {
        override fun onInputDeviceAdded(deviceId: Int) {}
        override fun onInputDeviceChanged(deviceId: Int) {}
        override fun onInputDeviceRemoved(deviceId: Int) { if (deviceId == padId) clearPad() }
      }
      im.registerInputDeviceListener(l, main)
      deviceListener = l
    }
    // a pad that is already paired shows up before any button is pressed
    for (id in im.inputDeviceIds) {
      val d = im.getInputDevice(id) ?: continue
      if (isPadDevice(d)) { padId = id; padName = d.name; emit(); break }
    }
  }

  private fun unhook() {
    val activity = appContext.currentActivity
    deviceListener?.let { (activity?.getSystemService(Context.INPUT_SERVICE) as? InputManager)?.unregisterInputDeviceListener(it) }
    deviceListener = null
    hookedWindow = null
  }

  private fun isPadDevice(d: InputDevice): Boolean {
    val s = d.sources
    return !d.isVirtual && ((s and InputDevice.SOURCE_GAMEPAD) == InputDevice.SOURCE_GAMEPAD ||
      (s and InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK)
  }

  private fun bitFor(code: Int): Int = when (code) {
    KeyEvent.KEYCODE_BUTTON_A -> 0
    KeyEvent.KEYCODE_BUTTON_B -> 1
    KeyEvent.KEYCODE_BUTTON_X -> 2
    KeyEvent.KEYCODE_BUTTON_Y -> 3
    KeyEvent.KEYCODE_BUTTON_L1 -> 4
    KeyEvent.KEYCODE_BUTTON_R1 -> 5
    KeyEvent.KEYCODE_BUTTON_L2 -> 6
    KeyEvent.KEYCODE_BUTTON_R2 -> 7
    KeyEvent.KEYCODE_BUTTON_SELECT, KeyEvent.KEYCODE_BACK -> 8
    KeyEvent.KEYCODE_BUTTON_START, KeyEvent.KEYCODE_MENU -> 9
    KeyEvent.KEYCODE_BUTTON_THUMBL -> 10
    KeyEvent.KEYCODE_BUTTON_THUMBR -> 11
    KeyEvent.KEYCODE_DPAD_UP -> 12
    KeyEvent.KEYCODE_DPAD_DOWN -> 13
    KeyEvent.KEYCODE_DPAD_LEFT -> 14
    KeyEvent.KEYCODE_DPAD_RIGHT -> 15
    KeyEvent.KEYCODE_BUTTON_MODE -> 16
    else -> -1
  }

  private fun onKey(e: KeyEvent): Boolean {
    val d = e.device ?: return false
    if (!isPadDevice(d)) return false   // phone back key, keyboards: untouched
    val bit = bitFor(e.keyCode)
    if (bit < 0) return false
    when (e.action) {
      KeyEvent.ACTION_DOWN -> keyBits = keyBits or (1 shl bit)
      KeyEvent.ACTION_UP -> keyBits = keyBits and (1 shl bit).inv()
      else -> return true
    }
    seen(d)
    emit()
    return true
  }

  private fun onMotion(e: MotionEvent): Boolean {
    val d = e.device ?: return false
    if ((e.source and InputDevice.SOURCE_JOYSTICK) != InputDevice.SOURCE_JOYSTICK || e.action != MotionEvent.ACTION_MOVE) return false
    val useZ = d.getMotionRange(MotionEvent.AXIS_Z, e.source) != null   // most pads: right stick on Z/RZ, some on RX/RY
    axes[0] = e.getAxisValue(MotionEvent.AXIS_X)
    axes[1] = e.getAxisValue(MotionEvent.AXIS_Y)
    axes[2] = e.getAxisValue(if (useZ) MotionEvent.AXIS_Z else MotionEvent.AXIS_RX)
    axes[3] = e.getAxisValue(if (useZ) MotionEvent.AXIS_RZ else MotionEvent.AXIS_RY)
    axes[4] = maxOf(e.getAxisValue(MotionEvent.AXIS_LTRIGGER), e.getAxisValue(MotionEvent.AXIS_BRAKE))
    axes[5] = maxOf(e.getAxisValue(MotionEvent.AXIS_RTRIGGER), e.getAxisValue(MotionEvent.AXIS_GAS))
    val hx = e.getAxisValue(MotionEvent.AXIS_HAT_X)
    val hy = e.getAxisValue(MotionEvent.AXIS_HAT_Y)
    hatBits = (if (hy < -0.5f) 1 shl 12 else 0) or (if (hy > 0.5f) 1 shl 13 else 0) or
      (if (hx < -0.5f) 1 shl 14 else 0) or (if (hx > 0.5f) 1 shl 15 else 0)
    seen(d)
    emit()
    return true
  }

  private fun seen(d: InputDevice) {
    padId = d.id
    padName = d.name
  }

  private fun clearPad() {
    axes.fill(0f)
    keyBits = 0
    hatBits = 0
    if (padId >= 0) {
      val im = appContext.currentActivity?.getSystemService(Context.INPUT_SERVICE) as? InputManager
      if (im?.getInputDevice(padId) == null) { padId = -1; padName = "" }
    }
    emit()
  }

  private fun emit() {
    sendEvent("onGamepad", mapOf(
      "connected" to (padId >= 0),
      "name" to padName,
      "axes" to axes.map { it.toDouble() },
      "buttons" to (keyBits or hatBits)
    ))
  }

  // ------------------------------------------------------------------ Wi-Fi
  private fun bindWifi(ssid: String, password: String) {
    val ctx = appContext.reactContext ?: return
    val cm = ctx.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
    unbindWifi()
    val req = NetworkRequest.Builder()
      .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
      .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
    if (ssid.isNotEmpty() && Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
      val spec = WifiNetworkSpecifier.Builder().setSsid(ssid)
      if (password.isNotEmpty()) spec.setWpa2Passphrase(password)
      req.setNetworkSpecifier(spec.build())   // system shows a one-tap "connect to <ssid>" dialog
    }
    val cb = object : ConnectivityManager.NetworkCallback() {
      override fun onAvailable(network: Network) {
        cm.bindProcessToNetwork(network)
        sendEvent("onWifi", mapOf("bound" to true, "reason" to "available"))
      }
      override fun onLost(network: Network) {
        cm.bindProcessToNetwork(null)
        sendEvent("onWifi", mapOf("bound" to false, "reason" to "lost"))
      }
      override fun onUnavailable() {
        sendEvent("onWifi", mapOf("bound" to false, "reason" to "unavailable"))
      }
    }
    try {
      cm.requestNetwork(req.build(), cb)
      netCallback = cb
    } catch (e: Exception) {
      sendEvent("onWifi", mapOf("bound" to false, "reason" to (e.message ?: "error")))
    }
  }

  private fun unbindWifi() {
    val ctx = appContext.reactContext ?: return
    val cm = ctx.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
    netCallback?.let { runCatching { cm.unregisterNetworkCallback(it) } }
    netCallback = null
    cm.bindProcessToNetwork(null)
  }
}
