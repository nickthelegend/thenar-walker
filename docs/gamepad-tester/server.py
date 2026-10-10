"""Thenar Pad Check web server.

    python docs/gamepad-tester/server.py          (or double-click START_PAD_CHECK.bat)

  PC:     http://localhost:8795           browsers only give controllers to localhost or https pages
  Phone:  https://<this PC's IP>:8796     same Wi-Fi; self-signed certificate -> tap "Advanced" -> "Proceed" once
The certificate is made on first run (needs the 'cryptography' package) and kept in .cert/ next to this file.
"""
import datetime
import http.server
import ipaddress
import os
import socket
import ssl
import sys
import threading
import webbrowser
from functools import partial

HERE = os.path.dirname(os.path.abspath(__file__))
HTTP_PORT, HTTPS_PORT = 8795, 8796


class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')   # always the latest page after an update
        super().end_headers()

    def log_message(self, fmt, *args):
        if args and str(args[1]) not in ('200', '304'):
            sys.stderr.write('%s  %s\n' % (self.address_string(), fmt % args))


def lan_ips():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))   # no packet is sent; picks the outgoing interface
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        ips.update(ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith('127.'))
    except OSError:
        pass
    return sorted(ips)


def make_cert(ips):
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.x509.oid import NameOID
    d = os.path.join(HERE, '.cert')
    crt, key = os.path.join(d, 'cert.pem'), os.path.join(d, 'key.pem')
    want = sorted(set(ips) | {'127.0.0.1'})
    if os.path.exists(crt) and os.path.exists(key):
        try:
            c = x509.load_pem_x509_certificate(open(crt, 'rb').read())
            have = sorted(str(i) for i in c.extensions.get_extension_for_class(x509.SubjectAlternativeName).value.get_values_for_type(x509.IPAddress))
            if set(want) <= set(have) and c.not_valid_after_utc > datetime.datetime.now(datetime.timezone.utc):
                return crt, key
        except Exception:
            pass
    os.makedirs(d, exist_ok=True)
    k = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, 'Thenar Pad Check (local)')])
    now = datetime.datetime.now(datetime.timezone.utc)
    san = [x509.DNSName('localhost')] + [x509.IPAddress(ipaddress.ip_address(i)) for i in want]
    c = (x509.CertificateBuilder().subject_name(name).issuer_name(name).public_key(k.public_key())
         .serial_number(x509.random_serial_number()).not_valid_before(now - datetime.timedelta(days=1))
         .not_valid_after(now + datetime.timedelta(days=825)).add_extension(x509.SubjectAlternativeName(san), critical=False)
         .sign(k, hashes.SHA256()))
    open(crt, 'wb').write(c.public_bytes(serialization.Encoding.PEM))
    open(key, 'wb').write(k.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8, serialization.NoEncryption()))
    return crt, key


def main():
    sys.stdout.reconfigure(line_buffering=True)   # show the addresses right away, even when not on a console
    handler = partial(Handler, directory=HERE)
    http_srv = http.server.ThreadingHTTPServer(('127.0.0.1', HTTP_PORT), handler)
    threading.Thread(target=http_srv.serve_forever, daemon=True).start()
    print(f'\n  Thenar Pad Check\n  PC:    http://localhost:{HTTP_PORT}')
    ips = lan_ips()
    try:
        crt, key = make_cert(ips)
        https_srv = http.server.ThreadingHTTPServer(('0.0.0.0', HTTPS_PORT), handler)
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(crt, key)
        https_srv.socket = ctx.wrap_socket(https_srv.socket, server_side=True)
        threading.Thread(target=https_srv.serve_forever, daemon=True).start()
        for ip in ips:
            print(f'  Phone: https://{ip}:{HTTPS_PORT}   (same Wi-Fi; accept the certificate warning once)')
    except ImportError:
        print('  Phone access off: pip install cryptography   (needed to make the https certificate)')
    except OSError as e:
        print(f'  Phone access off: {e}')
    print('\n  Ctrl+C or close this window to stop.\n')
    if '--no-browser' not in sys.argv:
        webbrowser.open(f'http://localhost:{HTTP_PORT}')
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
