@echo off
setlocal
if "%~1"=="" goto usage
set "ORT_ROOT=%~f1"
pushd "%~dp0.."
set "CPU_ROOT=%CD%"
popd
if not exist "%ORT_ROOT%\include\onnxruntime_cxx_api.h" (
  echo Missing ONNX Runtime C++ headers under "%ORT_ROOT%\include" 1>&2
  exit /b 1
)
if not exist "%ORT_ROOT%\lib\onnxruntime.lib" (
  echo Missing x64 ONNX Runtime import library under "%ORT_ROOT%\lib" 1>&2
  exit /b 1
)
if not exist "%ORT_ROOT%\lib\onnxruntime.dll" (
  echo Missing x64 ONNX Runtime DLL under "%ORT_ROOT%\lib" 1>&2
  exit /b 1
)
where cmake >nul 2>nul
if errorlevel 1 (
  echo CMake 3.20 or newer must be on PATH. 1>&2
  exit /b 1
)
where cl >nul 2>nul
if errorlevel 1 (
  echo Run this command in x64 Native Tools Command Prompt for Visual Studio 2022. 1>&2
  exit /b 1
)
if not defined CPU_BUILD_DIR set "CPU_BUILD_DIR=%CPU_ROOT%\build\cpu-windows"
if not defined CPU_INSTALL_DIR set "CPU_INSTALL_DIR=%CPU_ROOT%\cpu_runtime"
cmake -S "%CPU_ROOT%" -B "%CPU_BUILD_DIR%" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DR20_WITH_BPU=OFF -DR29_WITH_ORT=ON -DORT_ROOT="%ORT_ROOT%" -DPERCEPTION_BUILD_PRODUCT=OFF -DBUILD_TESTING=OFF
if errorlevel 1 exit /b 1
cmake --build "%CPU_BUILD_DIR%" --target dag_flow_worker visualize_saved_frame perception_sdk perception_sdk_demo
if errorlevel 1 exit /b 1
cmake --install "%CPU_BUILD_DIR%" --prefix "%CPU_INSTALL_DIR%"
if errorlevel 1 exit /b 1
echo CPU runtime ready: %CPU_INSTALL_DIR%
echo Configure explicit ONNX models and preprocessing in a copy of profiles\cpu_exact.json.
exit /b 0
:usage
echo Usage: tools\build_cpu_windows.cmd C:\path\onnxruntime-win-x64-1.16.3 1>&2
echo Use x64 Native Tools Command Prompt for Visual Studio 2022. 1>&2
exit /b 1
