@echo off
echo Select Deployment Target:
echo [1] Main Robot
echo [2] Second Robot (Custom Team Number)

set /p choice="Enter option (1 or 2): "

if "%choice%"=="1" (
    echo Deploying to Main Robot...
    call .\gradlew deploy
    goto end
)

if "%choice%"=="2" (
    set /p secondTeam="Enter second-robot team number: "
    echo Deploying to Team %secondTeam%...
    call .\gradlew deploy -PteamNumber=%secondTeam%
    goto end
)

echo Invalid option selected. Exiting...

:end