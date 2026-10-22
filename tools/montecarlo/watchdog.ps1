# Monte Carlo watchdog: keeps Windows awake and keeps the (resumable) WSL run going until
# runs\montecarlo\OVERNIGHT_DONE exists or 36 h have passed. Changes no settings.
# - run not active (script, WSL or the PC's session closed it) -> clean up, start again
# - run active but no new result line for 40 min (hung PX4/Gazebo) -> kill everything, start again
# overnight.sh skips completed runs, so every start continues with the first unfinished run.
# Start it outside the Claude app's process tree (WMI), e.g.:
#   Invoke-CimMethod Win32_Process Create @{CommandLine='powershell -NoProfile -WindowStyle Hidden -File <repo>\tools\montecarlo\watchdog.ps1'}
Add-Type -Name Pwr -Namespace W -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint f);'
$repoWin = "C:\Users\erina\Desktop\Yeageth\workplace\UAV"
$runs = "$repoWin\runs\montecarlo"
$done = "$runs\OVERNIGHT_DONE"
$log = "$runs\watchdog.log"
$cmd = 'cd /mnt/c/Users/erina/Desktop/Yeageth/workplace/UAV && bash tools/montecarlo/overnight.sh >> /root/mc_overnight.log 2>&1'
$end = (Get-Date).AddHours(36)
$stallMin = 40

function Write-Log($msg) { "$(Get-Date -Format s) $msg" | Add-Content $log }
function Get-LastResult {
    $f = Get-ChildItem "$runs\*\results.jsonl" -ErrorAction SilentlyContinue | Sort-Object LastWriteTime | Select-Object -Last 1
    if ($f) { $f.LastWriteTime } else { Get-Date }
}
function Clear-Sim {
    # brackets keep pkill from matching the bash -c line that runs it
    & wsl.exe -d Ubuntu-24.04 -u root -- bash -c "pkill -9 -f '[o]vernight.sh'; pkill -9 -f '[t]ools.montecarlo.run'; pkill -9 -f '[b]in/px4 -i'; pkill -9 -f '[g]z sim'; rm -f /tmp/px4_lock-* /tmp/px4-sock-*" | Out-Null
}

[W.Pwr]::SetThreadExecutionState([uint32]"0x80000001") | Out-Null
Write-Log "watchdog start"
$started = $null
while (-not (Test-Path $done) -and (Get-Date) -lt $end) {
    $alive = & wsl.exe -d Ubuntu-24.04 -u root -- pgrep -f "[t]ools/montecarlo/overnight.sh"
    $restart = $null
    if (-not $alive) {
        $restart = "run not active"
    } elseif ($started -and ((Get-Date) - $started).TotalMinutes -gt $stallMin -and
              ((Get-Date) - (Get-LastResult)).TotalMinutes -gt $stallMin) {
        $restart = "no new result for $stallMin min (hung)"
    }
    if ($restart) {
        if (-not (Test-Path $done)) {
            Write-Log "$restart -> cleaning up, starting (continues with the first unfinished run)"
            Clear-Sim
            Start-Process wsl.exe -ArgumentList @("-d", "Ubuntu-24.04", "-u", "root", "--", "bash", "-c", "`"$cmd`"") -WindowStyle Hidden
            $started = Get-Date
            Start-Sleep 90
        }
    }
    [W.Pwr]::SetThreadExecutionState([uint32]"0x80000001") | Out-Null
    Start-Sleep 60
}
Write-Log "watchdog end (done: $(Test-Path $done))"
[W.Pwr]::SetThreadExecutionState([uint32]"0x80000000") | Out-Null
