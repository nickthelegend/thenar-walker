@echo off
rem Thenar arm calibrator: serves the page on http://localhost:8791 (Web Serial needs localhost or https) and opens Edge.
cd /d "%~dp0"
cd ..\..
start "" msedge "http://localhost:8791/docs/calibrator/"
python -m http.server 8791 --bind 127.0.0.1
pause
