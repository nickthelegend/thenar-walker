# Wiring: Raspberry Pi brain (planned; confirm on the real robot)

The ESP32-S3 wiring is in `docs/WIRING.md` (out of date: it still shows classic-ESP32 pins). This is the Pi version.
BCM GPIO numbers, with the physical header pin in brackets. All grounds are common.

| From (Pi) | To | Notes |
|---|---|---|
| 3V3 [1] | PCA9685 VCC | logic only (3.3 V is fine for the PCA9685) |
| GPIO2 SDA [3] | PCA9685 SDA, ADS1115 SDA | I2C-1, 400 kHz is fine |
| GPIO3 SCL [5] | PCA9685 SCL, ADS1115 SCL | |
| GPIO17 [11] | PCA9685 OE | active low; the runtime holds it HIGH (servos off) until a joint is switched on |
| GPIO27 [13] | status LED (+330 Ω) | lit = phone linked, 2 Hz blink = waiting |
| GPIO5 [29] | MDD10A DIR1 | left motor pair |
| GPIO6 [31] | MDD10A DIR2 | right motor pair |
| GPIO12 [32] | MDD10A PWM1 | hardware PWM0 (`dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4`), 20 kHz |
| GPIO13 [33] | MDD10A PWM2 | hardware PWM1 |
| GND [6, 9, 14, …] | PCA9685 GND, MDD10A GND, ADS1115 GND, buck GND | common ground |

| Power | Goes to |
|---|---|
| Battery (≤ 24 V) → KCD4 kill switch | everything below |
| → MDD10A power input | motors |
| → 6 V high-current buck (≥ 10 A) | PCA9685 V+ (servo power), never the Pi |
| → 5 V 5 A buck | Pi 5 USB-C (or the 5 V header pins with a proper supply) |
| → 100k / 27k divider → ADS1115 AIN0 | battery sense (optional) |

Local parts (India): ADS1115 module (Robu ~₹350), 5 V 5 A buck (XL4015 or a 5 A USB-C PD buck, ~₹300–600),
Pi 5 active cooler (~₹500). Check prices on the day.
