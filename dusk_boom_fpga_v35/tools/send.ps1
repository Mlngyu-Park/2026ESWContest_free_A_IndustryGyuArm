# send.ps1 - upload a file prepared by xfer_prep.py through the serial console (board runs /tmp/rx.py)
param([string]$Port="COM7",[int]$Baud=115200,[string]$Lines)
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,'None',8,'One'
$p.ReadTimeout = 3000; $p.WriteTimeout = 10000; $p.NewLine = "`n"; $p.DtrEnable = $false; $p.RtsEnable = $false
$p.Open(); $p.DiscardInBuffer()
$all = [System.IO.File]::ReadAllLines($Lines)
$cmd = $all[0].Substring(5)
$p.Write("stty -echo`n"); Start-Sleep -Milliseconds 500; $p.DiscardInBuffer()
$p.Write($cmd + "`n")
function WaitFor([string]$pat, [int]$ms) {
  $end = (Get-Date).AddMilliseconds($ms)
  while ((Get-Date) -lt $end) { try { $l = $p.ReadLine().Trim(); if ($l -match $pat) { return $l } } catch {} }
  return $null
}
if (-not (WaitFor 'READY$' 15000)) { Write-Output "NO READY"; $p.Close(); exit 1 }
$t0 = Get-Date; $i = 1; $retries = 0; $n = $all.Length - 1
while ($i -le $n) {
  $p.Write($all[$i] + "`n")
  $r = WaitFor '(^|\s)[AN] \d+$' 5000
  if ($r -and $r -match ('(^|\s)A ' + ($i - 1) + '$')) { $i++ } else { $retries++; if ($retries -gt 50) { Write-Output "TOO MANY RETRIES at $i ($r)"; break } }
  if (($i % 200) -eq 0) { Write-Output ("  {0}/{1} chunks  {2:N0}s" -f $i, $n, ((Get-Date) - $t0).TotalSeconds) }
}
$p.Write("END`n")
$res = WaitFor 'RX_(OK|FAIL)' 20000
$p.Write("stty echo`n"); Start-Sleep -Milliseconds 300
$p.Close()
Write-Output ("{0}  ({1} chunks, {2} retries, {3:N0}s)" -f $res, $n, $retries, ((Get-Date) - $t0).TotalSeconds)
