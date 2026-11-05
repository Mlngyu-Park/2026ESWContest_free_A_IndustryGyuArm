# reboot_capture.ps1 - send "reboot" at $FromBaud, then record the console at 115200 for $Seconds
param([string]$Port="COM7",[int]$FromBaud=921600,[int]$Seconds=90,[string]$Out="C:\work\fpga_v35\tools\boot_log.txt",[string]$Cmd="sync; reboot")
$p = New-Object System.IO.Ports.SerialPort $Port,$FromBaud,'None',8,'One'
$p.Open(); $p.Write("`n" + $Cmd + "`n"); Start-Sleep -Milliseconds 300; $p.Close()
$p = New-Object System.IO.Ports.SerialPort $Port,115200,'None',8,'One'
$p.ReadTimeout = 200; $p.Open()
$end = (Get-Date).AddSeconds($Seconds); $buf = New-Object System.Text.StringBuilder
while ((Get-Date) -lt $end) { try { [void]$buf.Append($p.ReadExisting()) } catch {} ; Start-Sleep -Milliseconds 100 }
$p.Close()
[System.IO.File]::WriteAllText($Out, $buf.ToString())
Write-Output $buf.ToString()
