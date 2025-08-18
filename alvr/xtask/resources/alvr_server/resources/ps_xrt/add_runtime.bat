@echo off

reg delete HKLM\SOFTWARE\Khronos\OpenXR\1 /v ActiveRuntime
set dirpath=%~dp0
set json=picostreaming-openxr.json
set jsonpath=%dirpath%%json%
echo add apilayer , jsonpath=%jsonpath%

reg add HKLM\SOFTWARE\Khronos\OpenXR\1 /v ActiveRuntime /t REG_SZ /d "%jsonpath%" /f 
reg add HKLM\SOFTWARE\Khronos\OpenXR\1\AvailableRuntimes /v "%jsonpath%" /t REG_DWORD /d 0 /f 
pause
 