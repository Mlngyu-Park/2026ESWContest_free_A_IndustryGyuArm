@echo off
cd /d C:\work\fpga_v35\vivado
call C:\Xilinx\Vivado\2020.2\settings64.bat
call vivado -mode batch -nojournal -log build.log -source build.tcl
echo VIVADO_EXIT=%ERRORLEVEL%
