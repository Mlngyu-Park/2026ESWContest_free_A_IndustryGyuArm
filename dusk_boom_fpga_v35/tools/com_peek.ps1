param([string]$Port="COM7",[int]$Baud=115200,[int]$Seconds=6,[string]$Send="")
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,'None',8,'One'
$p.ReadTimeout = 200; $p.DtrEnable = $false; $p.RtsEnable = $false
$p.Open()
if ($Send -ne "") { $p.Write($Send.Replace('\r',"`r").Replace('\n',"`n")) }
$end = (Get-Date).AddSeconds($Seconds); $buf = ""
while ((Get-Date) -lt $end) { try { $buf += $p.ReadExisting() } catch {} ; Start-Sleep -Milliseconds 100 }
$p.Close()
Write-Output "----- $Port received $($buf.Length) chars -----"
Write-Output $buf
