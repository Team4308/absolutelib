@echo off
setlocal enabledelayedexpansion

:: ========================================================
:: CREATE LOCAL BACKUP CACHE
:: ========================================================

echo Creating local safety backup...

set BACKUP_DIR=.git-backup-cache
if not exist %BACKUP_DIR% mkdir %BACKUP_DIR%

for /f "tokens=1-4 delims=/: " %%a in ("%date%") do (
    for /f "tokens=1-3 delims=:." %%i in ("%time%") do (
        set TS=%%d-%%b-%%c-%%i-%%j-%%k
    )
)

set CACHE_PATH=%BACKUP_DIR%\%TS%
mkdir "%CACHE_PATH%"

echo Backing up working directory to %CACHE_PATH% ...

:: Copy all files except .git
xcopy * "%CACHE_PATH%\" /E /I /H /Y /EXCLUDE:.git-exclude-list.txt >nul

echo Backup completed.
echo.


:: ========================================================
:: GIT OPS START
:: ========================================================

:: --- 1. Store current branch ---
for /f "tokens=*" %%b in ('git rev-parse --abbrev-ref HEAD') do set CURRENT_BRANCH=%%b
echo Current branch: %CURRENT_BRANCH%

:: --- 2. Ask for commit message ---
set /p COMMIT_MSG="Enter commit message: "
echo.


:: ========================================================
:: MAIN BRANCH COMMIT + FORCE PUSH
:: ========================================================

echo === Committing MAIN branch first ===

git reset
git add .
git restore --staged site >nul 2>&1

git commit -m "%COMMIT_MSG%"
git push origin %CURRENT_BRANCH% --force


:: ========================================================
:: GH-PAGES DEPLOY
:: ========================================================

echo.
echo === Deploying to gh-pages ===

git fetch
git branch --list gh-pages >nul 2>&1
if %errorlevel%==0 (
    git checkout gh-pages
) else (
    git checkout -b gh-pages
)

git reset
git rm -r --cached . >nul 2>&1

if exist site (
    git add site/*
) else (
    echo ERROR: /site folder does not exist
    pause
    exit /b
)

git commit -m "%COMMIT_MSG%"
git push origin gh-pages --force

git checkout %CURRENT_BRANCH%

echo.
echo ------------------------------------
echo Deployment Complete!
echo Local backup stored at: %CACHE_PATH%
echo ------------------------------------
pause
