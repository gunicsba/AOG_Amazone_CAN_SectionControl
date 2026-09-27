@echo off
echo Building AOG-Amazone-PCAN...
pip install -r "%~dp0requirements.txt" pyinstaller
if exist "%~dp0icon.ico" (
    set ICON_FLAG=--icon="%~dp0icon.ico"
) else (
    set ICON_FLAG=
    echo WARNING: icon.ico not found!
)
python -m PyInstaller --onefile --console --name "AOG-Amazone-PCAN" %ICON_FLAG% ^
    --collect-submodules can.interfaces.pcan --copy-metadata python-can ^
    "%~dp0AOG_Amazone_PCAN_bridge.py"
if exist "%~dp0dist\AOG-Amazone-PCAN.exe" (
    copy "%~dp0dist\AOG-Amazone-PCAN.exe" "%~dp0AOG-Amazone-PCAN.exe" >nul
    echo.
    echo Built: AOG-Amazone-PCAN.exe
)
pause
