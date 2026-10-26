"""Make the Pi the robot's Wi-Fi access point, like the ESP32 build: SSID "<Team>_TF", WPA2, the phone opens
http://192.168.4.1 . Uses NetworkManager (Raspberry Pi OS Bookworm). Run on the Pi:

    sudo python3 pi/tools/setup_ap.py "Thenar Walker"          (asks for the password; never stored in git)
    sudo python3 pi/tools/setup_ap.py --off                    (back to normal Wi-Fi)

The password is read from THENAR_WIFI_PASSWORD or typed in; it goes only into NetworkManager's own config
(/etc/NetworkManager/system-connections, root-only).
"""
import argparse
import getpass
import os
import subprocess
import sys

CON = "thenar-ap"


def nmcli(*args: str, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(["nmcli", *args], check=check, text=True, capture_output=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("team", nargs="?", default="Thenar Walker")
    ap.add_argument("--off", action="store_true")
    ap.add_argument("--ifname", default="wlan0")
    a = ap.parse_args()
    if os.geteuid() != 0:
        print("run with sudo", file=sys.stderr)
        return 1
    if a.off:
        nmcli("connection", "down", CON, check=False)
        nmcli("connection", "delete", CON, check=False)
        print("access point removed")
        return 0
    ssid = a.team.replace(" ", "")[:20] + "_TF"   # same rule as tw::device_name: spaces dropped, 20 chars, + "_TF"
    pw = os.environ.get("THENAR_WIFI_PASSWORD") or getpass.getpass(f"WPA2 password for {ssid} (8-63 chars): ")
    if not 8 <= len(pw) <= 63:
        print("password must be 8-63 characters", file=sys.stderr)
        return 1
    nmcli("connection", "delete", CON, check=False)
    nmcli("connection", "add", "type", "wifi", "ifname", a.ifname, "con-name", CON, "autoconnect", "yes", "ssid", ssid)
    nmcli("connection", "modify", CON, "802-11-wireless.mode", "ap", "802-11-wireless.band", "bg",
          "802-11-wireless.channel", "6", "802-11-wireless.powersave", "2",          # 2 = power save off (low latency)
          "ipv4.method", "shared", "ipv4.addresses", "192.168.4.1/24", "ipv6.method", "disabled",
          "wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.proto", "rsn", "wifi-sec.pairwise", "ccmp", "wifi-sec.psk", pw)
    nmcli("connection", "up", CON)
    print(f"access point {ssid} up; phone: join it, mobile data OFF, open http://192.168.4.1")
    return 0


if __name__ == "__main__":
    sys.exit(main())
