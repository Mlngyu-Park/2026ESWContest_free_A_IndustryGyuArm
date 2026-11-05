# send lines to the serial console one by one, print everything received
param([string]$Port="COM7",[int]$Baud=115200,[string]$File="",[int]$Wait=1500,[int]$Tail=3000)
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,'None',8,'One'
$p.ReadTimeout = 200; $p.DtrEnable = $false; $p.RtsEnable = $false
$p.Open()
$buf = ""
foreach ($line in Get-Content $File) {
  if ($line -match '^#WAIT (\d+)') { $w=[int]$Matches[1]; $end=(Get-Date).AddMilliseconds($w); while((Get-Date) -lt $end){ $buf += $p.ReadExisting(); Start-Sleep -Milliseconds 100 }; continue }
  $p.Write($line + "`r")
  $end = (Get-Date).AddMilliseconds($Wait)
  while ((Get-Date) -lt $end) { $buf += $p.ReadExisting(); Start-Sleep -Milliseconds 100 }
}
$end = (Get-Date).AddMilliseconds($Tail)
while ((Get-Date) -lt $end) { $buf += $p.ReadExisting(); Start-Sleep -Milliseconds 100 }
$p.Close()
Write-Output $buf
