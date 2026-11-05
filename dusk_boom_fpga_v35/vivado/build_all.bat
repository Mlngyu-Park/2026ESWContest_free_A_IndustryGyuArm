@echo off
rem build_all.bat - DUSK_BOOM v3.5: Vivado build -> FSBL + BOOT.bin -> ..\sd_card
rem Double-click, or run from cmd. Finds Vivado / Vitis under C:\Xilinx or C:\AMDDesignTools
rem when they are not in PATH yet. Keep this folder on a short path WITHOUT Korean characters
rem (e.g. C:\work\fpga_v35) - Vivado fails on non-ASCII paths.
setlocal
cd /d "%~dp0"
where vivado >nul 2>nul || for /d %%D in (C:\Xilinx\Vivado\* C:\AMDDesignTools\*\Vivado) do if exist "%%D\settings64.bat" call "%%D\settings64.bat"
where xsct >nul 2>nul || for /d %%D in (C:\Xilinx\Vitis\* C:\AMDDesignTools\*\Vitis) do if exist "%%D\settings64.bat" call "%%D\settings64.bat"
where vivado >nul 2>nul || goto :novivado
call vivado -mode batch -nojournal -nolog -source build.tcl || goto :fail
where xsct >nul 2>nul || goto :noxsct
call xsct make_boot.tcl || goto :fail
echo.
echo OK - copy every file in ..\sd_card to the microSD (FAT32)
goto :end
:noxsct
echo.
echo Vivado build OK (system.xsa, dusk_boom_v35_top.bit) but xsct (Vitis) not found.
echo Copy vivado\system.xsa to the DGX and run linux/make_sd_from_xsa.sh there.
goto :end
:novivado
echo vivado not found - open a "Vivado Tcl Shell" or edit the path to settings64.bat
goto :fail
:fail
echo.
echo FAILED - see the messages above
pause
exit /b 1
:end
pause
