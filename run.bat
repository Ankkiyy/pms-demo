@echo off
setlocal

rem Switch to the repository root
pushd "%~dp0" || goto :error

rem Enter backend and activate the virtual environment
cd /d backend || goto :error

set "VENV_DIR=venv"
if exist ".venv\Scripts\activate.bat" set "VENV_DIR=.venv"

set "VENV_ACT=%VENV_DIR%\Scripts\activate.bat"
if not exist "%VENV_ACT%" (
	if exist "%VENV_DIR%\Scripts\activate" (
		set "VENV_ACT=%VENV_DIR%\Scripts\activate"
	) else (
		echo Virtual environment not found at "%CD%\%VENV_DIR%\Scripts\activate(.bat)"
		goto :error
	)
)

call "%VENV_ACT%" || goto :error

rem Launch FastAPI with host binding for the LAN and auto-reload
python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

rem Clean up when server stops
call deactivate
popd
exit /b 0

:error
echo Failed to start the backend. Ensure the virtual environment exists and dependencies are installed.
if defined VIRTUAL_ENV call deactivate
popd
exit /b 1