"""Phone-sized shopping sheets (HTML) for the local electronics shop — rover parts only.
Photos are the shops' own product photos (Robu.in), shown by link, not copied.
  python docs/shop/make_shop_sheets.py   -> docs/shop/sheet_1.html .. sheet_3.html
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
S3 = 'https://robu-prod-media.s3.ap-south-1.amazonaws.com/'

SHEETS = [
    ('1 / 4 — Motors and boards', [
        ('uploads/2018/11/chall.12-2.jpg', '25GA-370 gear motor, 12 V, 60 RPM', '5',
         '25 mm round metal gearbox, 4 mm D-shaft (also called JGA25-370 / GA25-370)',
         'NOT 100 RPM · NOT Johnson / BO / yellow motors'),
        ('uploads/2019/05/A7.jpg', 'Cytron MDD3A motor driver', '2',
         'Dual channel, 3 A, 4–16 V',
         'If not available: TB6612FNG module × 2 (below). NOT L298N'),
        ('uploads/2017/09/1-pc-Dual-Motor-Driver-1A-font-b-TB6612FNG-b-font-for-font-b-Arduino-b-800x800.jpg',
         'TB6612FNG dual motor driver module', '2',
         'ONLY if MDD3A is not available', 'Small red or blue board, motor supply up to 13.5 V'),
        ('uploads/2022/01/1-1.jpg', 'ESP32 development board, 38 pin', '1',
         'ESP32-WROOM-32 / DevKitC, USB (Micro or Type-C)', 'NOT ESP8266 / NodeMCU'),
        ('uploads/2017/09/1pcs-16-Channel-12-bit-PWM-Servo-Driver-I2C-interface-PCA9685-for-Arduino-Raspberry-Pi-DIY.jpg',
         'PCA9685 16-channel servo driver', '1', 'I2C, for the 6 arm servos', 'skip if you already have one'),
    ]),
    ('2 / 4 — Battery and safety', [
        ('uploads/2025/06/23741-1.jpg', 'LiPo battery 3S 11.1 V 2200 mAh 30C', '1',
         'XT60 plug, size max 106 × 34 × 26 mm', 'NOT 4S · 2200–2700 mAh is fine'),
        ('uploads/2015/02/37039963-152b-4724-902e-5144eb3b1dd3.jpg', 'LiPo balance charger (iMAX B6 / B3)', '1',
         'Must balance-charge 3S LiPo', '+ a LiPo safe bag if they have it'),
        ('uploads/2022/06/High-voltage-KCD4-Red-220V-16A-DPST-ON-OFF-4-Pin-Rocker-Switch-2-1.jpg',
         'KCD4 rocker switch, red, 4 pin', '1', '16 A or more, ON/OFF (the kill switch)', 'Size about 30 × 22 mm'),
        ('uploads/2026/01/1-36.jpg', 'Blade fuse holder + 15 A blade fuse', '1 + 2 fuses',
         'Inline (wire) type is best', 'Car accessory shops have this'),
    ]),
    ('3 / 4 — Power converters and plugs', [
        ('product/3444472/VGd1vXpq7LQKfNPdknbCvNAMxoaHVYs8RinGlUL9.webp', 'XT60 connector pair (male + female)', '2 pairs',
         'Amass or similar, 35 A', ''),
        ('uploads/2023/12/26-3.jpg', 'Buck converter XL4016, 8 A (or 10–20 A)', '1',
         'Adjustable step-down, input 12 V → output 6 V', 'For the arm servos — 8 A or more'),
        ('uploads/2023/07/1681309-1.jpg', 'Mini MP1584 buck converter', '1',
         'Adjustable 3 A step-down (set to 5 V)', 'For the ESP32'),
    ]),
    ('4 / 4 — Wires and small parts', [
        ('uploads/2024/10/High-Quality-Ultra-Flexible-16AWG-Silicone-Wire-red-1.png', 'Silicone wire 16 AWG', '1 m red + 1 m black',
         'Battery → switch → drivers', 'Also 22 AWG: 2 m red + 2 m black (motors)'),
        ('product/3447119/ZlHsR1u5yH1Wie1varNKyR7txAwDlUYtwBgpMBfb.webp', 'M3 brass heat-set inserts', '4 (small pack OK)',
         'Knurled, M3, about 5 mm long', ''),
        ('uploads/2025/09/0.1-40.jpg', 'M2.5 brass standoff, female-female', '2',
         '10 mm long (+ 2 × M2.5 screws)', ''),
    ]),
]

TEXT_ONLY = {
    '4 / 4 — Wires and small parts': [
        ('Resistors', '1 × 100 kΩ + 1 × 27 kΩ (¼ W)'),
        ('Jumper wires', '20 × female-female + servo extension leads'),
        ('Heat-shrink + zip ties', '1 small pack each'),
        ('Screws (hardware shop)', 'M3×6 × 8 · M3×10 × 4 · M3×8 × 6 · M3 nyloc nuts × 6 · M3×5 grub screws × 4 · '
                                   'M3×20 × 4 · M2.5×6 self-tapping × 20'),
    ]}

CSS = '''
body{margin:0;background:#ffffff;font-family:"Segoe UI",Arial,sans-serif;color:#13233a}
.page{width:100%;padding:16px;box-sizing:border-box}
.top{background:#12355b;color:#fff;border-radius:14px;padding:14px 20px;margin-bottom:14px}
.top h1{margin:0;font-size:30px}.top p{margin:4px 0 0;font-size:19px;opacity:.9}
.card{display:flex;gap:16px;border:2px solid #d7dee8;border-radius:14px;padding:10px;margin-bottom:11px;align-items:center}
.card img{width:165px;height:165px;object-fit:contain;border-radius:10px;background:#f4f6f9;flex:none}
.name{font-weight:700;font-size:25px;line-height:1.15}
.qty{display:inline-block;background:#d9622b;color:#fff;font-weight:700;border-radius:22px;padding:3px 14px;font-size:21px;margin:6px 0}
.spec{font-size:19px;margin-top:2px}.no{font-size:18px;color:#b0341b;margin-top:4px;font-weight:600}
.txt{border:2px dashed #c4ccd8;border-radius:14px;padding:10px 14px;margin-bottom:9px;font-size:19px}
.txt b{color:#12355b}
'''


def main():
    for i, (title, items) in enumerate(SHEETS, 1):
        cards = []
        for img, name, qty, spec, no in items:
            cards.append(f'<div class="card"><img src="{S3}{img}"><div><div class="name">{name}</div>'
                         f'<div class="qty">Qty: {qty}</div><div class="spec">{spec}</div>'
                         + (f'<div class="no">{no}</div>' if no else '') + '</div></div>')
        for name, spec in TEXT_ONLY.get(title, []):
            cards.append(f'<div class="txt"><b>{name}:</b> {spec}</div>')
        html = (f'<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Shop list {i}</title><style>{CSS}</style></head>'
                f'<body><div class="page"><div class="top"><h1>Robot parts list {title}</h1>'
                f'<p>Thenar Walker rover · please check stock &amp; price</p></div>{"".join(cards)}</div></body></html>')
        open(os.path.join(HERE, f'sheet_{i}.html'), 'w', encoding='utf-8').write(html)
    print('ok')


if __name__ == '__main__':
    main()
