@echo off
cd /d "%~dp0"
echo [SOL] 이전 서버가 있으면 정리하고 시작합니다...
for /f "tokens=5" %%p in ('netstat -ano ^| findstr :8765 ^| findstr LISTENING') do taskkill /F /PID %%p >nul 2>&1
python serve.py
