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
xcopy * "%CACHE_PATH%\" /E /I /H /Y /EXCLUDE:.git-exclude-list.txt >nul
echo Backup completed.
echo.


:: ========================================================
:: STORE CURRENT BRANCH
:: ========================================================

for /f "tokens=*" %%b in ('git rev-parse --abbrev-ref HEAD') do set CURRENT_BRANCH=%%b
echo Current branch: %CURRENT_BRANCH%
echo.

set /p COMMIT_MSG="Enter commit message: "
echo.


:: ========================================================
:: MAIN BRANCH COMMIT + FORCE PUSH
:: ========================================================

echo === Committing MAIN branch ===

git reset
git add .
git restore --staged site >nul 2>&1

git commit -m "%COMMIT_MSG%"
git push origin %CURRENT_BRANCH% --force

echo Main branch pushed.
echo.


:: ========================================================
:: PUSH SITE CONTENT TO GH-PAGES WITHOUT CHECKOUT
:: ========================================================

echo === Deploying site/* to origin/gh-pages WITHOUT checkout ===

:: Create a temporary index
set TEMP_INDEX=%TEMP%\ghpages_index_%RANDOM%.tmp
set TEMP_TREE=%TEMP%\ghpages_tree_%RANDOM%.tmp

set GIT_INDEX_FILE=%TEMP_INDEX%

:: Reset temp index
git read-tree --empty

:: Add only site/*
if exist site (
    git add site/*
) else (
    echo ERROR: site folder missing.
    pause
    exit /b
)

:: Create a tree object from the temporary index
git write-tree > "%TEMP_TREE%"
set /p TREE_HASH=<"%TEMP_TREE%"

echo Using tree %TREE_HASH% for gh-pages deployment...

:: Commit object for gh-pages
set AUTHOR_NAME=temp
set AUTHOR_EMAIL=temp@local

git commit-tree %TREE_HASH% -m "%COMMIT_MSG%" > "%TEMP_TREE%_commit"
set /p COMMIT_HASH=<"%TEMP_TREE%_commit"

:: Force push the commit to origin/gh-pages
git push origin %COMMIT_HASH%:refs/heads/gh-pages --force

echo GH-PAGES pushed without checkout.
echo.


:: Cleanup temp index
set GIT_INDEX_FILE=
del "%TEMP_INDEX%" >nul 2>&1
del "%TEMP_TREE%"* >nul 2>&1


echo ------------------------------------
echo Deployment complete!
echo Local backup stored at: %CACHE_PATH%
echo ------------------------------------
pause