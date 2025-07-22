@echo off
echo Starting 3 boat simulators in a triangle formation...

set SITL_EXE="%USERPROFILE%\Documents\Mission Planner\sitl\ArduRover.exe"
set PARAMS_FILE="%USERPROFILE%\Documents\Mission Planner\sitl\default_params\motorboat_flat.parm"

REM --- Home locations for each boat to form a triangle ---
REM Format: lat,lon,alt,heading
REM Base location is near Inzai, Chiba. Each boat is offset slightly.
set "HOME_1=35.877083,140.339553,0,0"
set "HOME_2=35.877283,140.339853,0,120"
set "HOME_3=35.876883,140.339853,0,240"

echo Launching SITL 1 (SYSID 1) at %HOME_1%
start "SITL 1" %SITL_EXE% --model motorboat --instance 0 --sysid 1 --defaults %PARAMS_FILE% --home %HOME_1%
timeout /t 3 >nul

echo Launching SITL 2 (SYSID 2) at %HOME_2%
start "SITL 2" %SITL_EXE% --model motorboat --instance 1 --sysid 2 --defaults %PARAMS_FILE% --home %HOME_2%
timeout /t 3 >nul

echo Launching SITL 3 (SYSID 3) at %HOME_3%
start "SITL 3" %SITL_EXE% --model motorboat --instance 2 --sysid 3 --defaults %PARAMS_FILE% --home %HOME_3%

echo All simulators launched.

