$p = New-Object System.IO.Ports.SerialPort "COM7",115200,'None',8,'One'
$p.ReadTimeout = 1000; $p.NewLine = "`n"
$p.Open(); $p.DiscardInBuffer()
$p.Write("python3 /tmp/rx.py /tmp/zz 1 x`n")
$end = (Get-Date).AddSeconds(4)
while ((Get-Date) -lt $end) { try { $l = $p.ReadLine(); Write-Output ("LINE[" + $l.Trim() + "] match=" + ($l.Trim() -match '^READY')) } catch { Write-Output "timeout" } }
$p.Write("END`n"); Start-Sleep -Milliseconds 800; Write-Output $p.ReadExisting()
$p.Close()
