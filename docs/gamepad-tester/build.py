"""tester.html is the page body (published as an artifact, which adds its own <head>).
index.html is the same page as a standalone file you can open straight in Chrome:  python docs/gamepad-tester/build.py"""
import os
here = os.path.dirname(os.path.abspath(__file__))
body = open(os.path.join(here, 'tester.html'), encoding='utf-8').read()
with open(os.path.join(here, 'index.html'), 'w', encoding='utf-8', newline='\n') as f:
    f.write('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
            '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
            '<!-- generated from tester.html by build.py -->\n</head>\n<body>\n' + body + '\n</body>\n</html>\n')
print('wrote index.html')
