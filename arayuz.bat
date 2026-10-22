@echo off
rem Simulasyonu (PX4 + Gazebo) ve tarayici yer kontrol arayuzunu birlikte baslatir.
rem   arayuz.bat            varsayilan dunya
rem   arayuz.bat karabuk    Karabuk dunyasi
rem   arayuz.bat -          sadece arayuz (simulasyon zaten acik)
title P1 yer kontrol arayuzu
set WORLD=%1
if "%WORLD%"=="" set WORLD=default
set REPO=/mnt/c/Users/erina/Desktop/Yeageth/workplace/UAV

if not "%WORLD%"=="-" start "P1 SITL (PX4 + Gazebo)" wsl -d Ubuntu-24.04 -u root -- env WORLD=%WORLD% bash %REPO%/px4/run_sitl.sh
start "P1 yer kontrol sunucusu" wsl -d Ubuntu-24.04 -u root --cd %REPO% -- python3 -m tools.gcs
echo Arayuz birkac saniye icinde tarayicida acilacak: http://localhost:8080
timeout /t 8 /nobreak >nul
start "" http://localhost:8080
