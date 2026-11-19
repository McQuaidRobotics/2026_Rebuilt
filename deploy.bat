@echo off
echo Select Target Robot:
echo [1] Team 3173
echo [2] Team 9999
set /p bot="Enter choice (1 or 2): "

if "%bot%"=="1" (
    echo Deploying to 3173...
    call .\gradlew deploy -PteamNumber=3173
)

if "%bot%"=="2" (
    echo Deploying to 9999...
    call .\gradlew deploy -PteamNumber=9999
)

pause