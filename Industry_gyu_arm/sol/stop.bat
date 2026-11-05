@echo off
for /f "tokens=5" %%p in ('netstat -ano ^| findstr :8765 ^| findstr LISTENING') do taskkill /F /PID %%p
echo [SOL] 종료됨
pause
