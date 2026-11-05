@echo off
REM SOL.exe 만들기 (윈도우에서 실행. 파이썬+pip 필요). 결과: dist\SOL.exe
cd /d "%~dp0"
pip install pyinstaller
pyinstaller --onefile --name SOL --console --add-data "chips.txt;." --add-data "physics.txt;." --add-data "pe2.txt;." --add-data "viewer_template.html;." --add-data "sol.py;." serve.py
echo.
echo dist\SOL.exe 완성. 더블클릭 = 브라우저 열림. 탭이나 검은 창을 닫으면 완전 종료.
pause
