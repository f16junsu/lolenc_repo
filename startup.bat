@echo off

REM Start the first PowerShell window
start powershell -NoExit -ExecutionPolicy ByPass -Command "& 'C:\Users\user\anaconda3\shell\condabin\conda-hook.ps1'; cd C:\Users\user\Desktop\RFSoC\lolenc_repo\; lolenc_master --configuration .\configuration.json --bind 127.0.0.3 --repository C:\Users\user\Desktop\RFSoC\lolenc_repo"

REM Start the second PowerShell window
start powershell -NoExit -ExecutionPolicy ByPass -Command "& 'C:\Users\user\anaconda3\shell\condabin\conda-hook.ps1'; cd C:\Users\user\Desktop\RFSoC\artiq-proxy\; python -m uvicorn main:app --reload --host 0.0.0.0 --port 8000"

REM Start the third PowerShell window
start powershell -NoExit -ExecutionPolicy ByPass -Command "& 'C:\Users\user\anaconda3\shell\condabin\conda-hook.ps1'; cd C:\Users\user\Desktop\RFSoC\iquip\; qiwis -c config.json"