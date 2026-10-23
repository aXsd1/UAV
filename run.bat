@echo off
title VTOL P1 SITL (PX4 + Gazebo)
echo ========================================================
echo   VTOL P1 SITL (PX4 + Gazebo Harmonic) Baslatiliyor...
echo ========================================================
echo.
wsl -d Ubuntu-24.04 -u root bash /mnt/c/Users/erina/Desktop/Yeageth/workplace/UAV/px4/run_sitl.sh
pause
