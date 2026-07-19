@echo off
chcp 65001 >nul
setlocal EnableExtensions
cd /d "%~dp0"

set "CONDA_ENV=pc_host"

echo ========================================
echo  Wheel-Legged PC Host 一键启动
echo ========================================
echo [info] PC 不跑 frpc，直接连服务器 6004/6005/6006
echo [info] （隧道由 NUC 侧 frpc 注册）

echo [start] 激活 conda 环境: %CONDA_ENV%

where conda >nul 2>&1
if errorlevel 1 (
  echo [error] 找不到 conda，请先安装 Anaconda/Miniconda 并勾选加入 PATH
  pause
  exit /b 1
)

for /f "delims=" %%i in ('conda info --base 2^>nul') do set "CONDA_BASE=%%i"
if defined CONDA_BASE (
  if exist "%CONDA_BASE%\Scripts\activate.bat" (
    call "%CONDA_BASE%\Scripts\activate.bat" %CONDA_ENV%
  ) else (
    call conda activate %CONDA_ENV%
  )
) else (
  call conda activate %CONDA_ENV%
)
if errorlevel 1 (
  echo [error] 无法激活环境 "%CONDA_ENV%"
  echo         请确认已创建: conda create -n pc_host python=3.12
  pause
  exit /b 1
)

echo [start] 启动上位机: python -m app.main
echo [start] 连接目标见 config\default.yaml （当前 118.190.106.114:6004/6005/6006）
echo ----------------------------------------
python -m app.main
set "EXITCODE=%ERRORLEVEL%"

echo ----------------------------------------
echo [exit] 上位机已退出 code=%EXITCODE%
pause
exit /b %EXITCODE%
