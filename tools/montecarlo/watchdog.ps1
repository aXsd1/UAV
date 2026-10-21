# Monte Carlo watchdog: keeps Windows awake and restarts the (resumable) WSL run if it stops,
# until runs\montecarlo\OVERNIGHT_DONE exists or 16 h have passed. Changes no settings.
Add-Type -Name Pwr -Namespace W -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint f);'
$repoWin = "C:\Users\erina\Desktop\Yeageth\workplace\UAV"
$done = "$repoWin\runs\montecarlo\OVERNIGHT_DONE"
$log = "$repoWin\runs\montecarlo\watchdog.log"
$cmd = 'cd /mnt/c/Users/erina/Desktop/Yeageth/workplace/UAV && bash tools/montecarlo/overnight.sh >> /root/mc_overnight.log 2>&1'
$end = (Get-Date).AddHours(16)
[W.Pwr]::SetThreadExecutionState([uint32]"0x80000001") | Out-Null
"$(Get-Date -Format s) watchdog start" | Add-Content $log
while (-not (Test-Path $done) -and (Get-Date) -lt $end) {
    $alive = & wsl.exe -d Ubuntu-24.04 -u root -- pgrep -f "tools/montecarlo/overnight.sh"
    if (-not $alive) {
        "$(Get-Date -Format s) run not active -> starting (resumes where it stopped)" | Add-Content $log
        Start-Process wsl.exe -ArgumentList @("-d", "Ubuntu-24.04", "-u", "root", "--", "bash", "-c", "`"$cmd`"") -WindowStyle Hidden
        Start-Sleep 90
    }
    Start-Sleep 60
}
"$(Get-Date -Format s) watchdog end (done: $(Test-Path $done))" | Add-Content $log
[W.Pwr]::SetThreadExecutionState([uint32]"0x80000000") | Out-Null

