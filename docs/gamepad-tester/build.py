"""tester.html is the page body (published as an artifact, which adds its own <head> and loads three.js from a CDN).
index.html is the same page as a standalone file for the local web server (server.py): three.js comes from vendor/,
so it also works with no internet.      python docs/gamepad-tester/build.py"""
import os
here = os.path.dirname(os.path.abspath(__file__))
body = open(os.path.join(here, 'tester.html'), encoding='utf-8').read()
body = body.replace('https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js', 'vendor/three.min.js')
body = body.replace('https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js', 'vendor/OrbitControls.js')
assert 'vendor/three.min.js' in body and 'vendor/OrbitControls.js' in body
with open(os.path.join(here, 'index.html'), 'w', encoding='utf-8', newline='\n') as f:
    f.write('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
            '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
            '<!-- generated from tester.html by build.py -->\n</head>\n<body>\n' + body + '\n</body>\n</html>\n')
print('wrote index.html')
