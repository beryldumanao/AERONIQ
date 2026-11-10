@echo off
setlocal
 title AeroNexus Command PRO
cd /d "%~dp0"
echo.
echo ============================================
echo       AeroNexus Command PRO - Launcher
echo ============================================
echo Folder: %CD%
echo.

set "PYTHON_CMD="
where py >nul 2>&1
if not errorlevel 1 (
    py -3.12 --version >nul 2>&1
    if not errorlevel 1 set "PYTHON_CMD=py -3.12"
)
if not defined PYTHON_CMD (
    echo [ERROR] Python 3.12 tidak ditemukan.
    echo Install Python 3.12 64-bit dari https://www.python.org/downloads/windows/
    echo Saat instalasi, centang Add Python to PATH.
    goto :finish
)
%PYTHON_CMD% --version
echo.
echo Memeriksa dependensi utama AeroNexus...
%PYTHON_CMD% -m pip install --upgrade pip
if errorlevel 1 (
    echo [ERROR] Gagal memperbarui pip.
    goto :finish
)
%PYTHON_CMD% -m pip install -r requirements.txt
if errorlevel 1 (
    echo [ERROR] Instalasi dependensi utama gagal.
    goto :finish
)
echo.
echo Menjalankan AeroNexus...
%PYTHON_CMD% -u aeronexus_pro.py
set "APP_EXIT=%ERRORLEVEL%"
echo.
if not "%APP_EXIT%"=="0" echo [ERROR] AeroNexus berhenti dengan kode %APP_EXIT%.
if "%APP_EXIT%"=="0" echo AeroNexus telah ditutup.
:finish
echo.
echo Jendela ini ditahan agar pesan error dapat dibaca.
pause
endlocal
