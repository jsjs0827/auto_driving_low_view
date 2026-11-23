@echo off
setlocal
call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
set "R33_SOURCE=%~dp0..\.."
cl /nologo /std:c++17 /utf-8 /EHsc /Zs "/I%R33_SOURCE%/shared/include" "%R33_SOURCE%/shared/examples/full_demo.cpp"
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /utf-8 /EHsc /Zs /DR33_DEMO_WITH_CAMERA=1 "/I%R33_SOURCE%/shared/include" "/I%R33_SOURCE%/stereo/include" "/I%R33_SOURCE%/src" "/I%R33_SOURCE%/support" "/I%R33_SOURCE%/support/native" "/I%R33_SOURCE%/framework/include" "%R33_SOURCE%/shared/examples/full_demo.cpp"
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /utf-8 /EHsc /Zs "/I%R33_SOURCE%/shared/include" "%R33_SOURCE%/shared/tests/full_sdk_contract.cpp"
exit /b %ERRORLEVEL%
