@echo off
setlocal
if "%~3"=="" (
  echo usage: build_native_windows.cmd build_dir opencv_library_root rapidjson_include_root
  exit /b 2
)
set "R33_RECHARGE_BUILD=%~1"
set "R33_RECHARGE_OPENCV=%~2"
set "R33_RECHARGE_JSON=%~3"
call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cmake -S "%~dp0.." -B "%R33_RECHARGE_BUILD%" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON "-DOpenCV_DIR=%R33_RECHARGE_OPENCV%/lib/cmake/opencv" "-DR33_RECHARGE_RAPIDJSON_ROOT=%R33_RECHARGE_JSON%" "-DR33_RECHARGE_RAPIDJSON_INCLUDE_DIR=%R33_RECHARGE_JSON%"
if errorlevel 1 exit /b 1
cmake --build "%R33_RECHARGE_BUILD%" --config Release
if errorlevel 1 exit /b 1
set "PATH=%R33_RECHARGE_OPENCV%/bin;%PATH%"
ctest --test-dir "%R33_RECHARGE_BUILD%" -C Release --output-on-failure
exit /b %ERRORLEVEL%
