@echo off
setlocal enabledelayedexpansion

:: Define registry paths
set "availableRuntimesPath=HKLM\SOFTWARE\Khronos\OpenXR\1\AvailableRuntimes"
set "activeRuntimePath=HKLM\SOFTWARE\Khronos\OpenXR\1"

:: Initialize variable
set "steamVRKey="

:: Get the key name containing SteamVR string
for /f "tokens=*" %%A in ('reg query "%availableRuntimesPath%" /s') do (
    ::echo loop key "%%A"
    echo "%%A" | findstr /i "steamxr_win64.json" >nul
    if !errorlevel! equ 0 (
        set "steamVRKey=%%A"
        echo found steamVRKey "%%A"
        goto :s_trim
    )
)
:s_trim
set "string=!steamVRKey! "

:: Define the substring to search for
set "substring=steamxr_win64.json"

:: Initialize variable to hold the left part of the string
set "leftPart="

:: Loop through each character in the string
for /l %%i in (0,1,255) do (
    set "char=!string:~%%i,1!"
    if "!char!"=="" goto :endloop
    set "leftPart=!leftPart!!char!"
    echo "!leftPart!" | findstr /i "%substring%" >nul
    if !errorlevel! equ 0 (
        set "leftPart=!leftPart:%substring%=!"
        set "leftPart=!leftPart!%substring%"
        goto :found
    )
)

:endloop
:: If the substring is not found
echo Substring "%substring%" not found in the string.
goto :eof

:found
:: Print the left part of the string including the substring
echo The left part of the string including "%substring%" is: "!leftPart!"

set "steamVRKey=!leftPart!"
 
:: Check if the key name containing SteamVR string is found
if defined steamVRKey (
    reg add "%activeRuntimePath%" /v ActiveRuntime /t REG_SZ /d "!steamVRKey!" /f
    echo ActiveRuntime is set to:\!steamVRKey!
) else (
    echo No key name containing SteamVR string found.
)

endlocal
pause
