@echo off
setlocal enabledelayedexpansion
title AbsoluteLib Release Script
color 0A

:: ========================================================
::  AbsoluteLib — Combined Release Script
::  Publishes code + builds & deploys GitHub Pages site
:: ========================================================

echo.
echo  =============================================
echo   AbsoluteLib Release Script
echo  =============================================
echo.

:: ========================================================
:: SAFETY CHECK 1: Must be inside a git repo
:: ========================================================
for /f "delims=" %%R in ('git rev-parse --show-toplevel 2^>nul') do set "REPO_ROOT=%%R"
if "%REPO_ROOT%"=="" (
    echo [ERROR] Not inside a git repository. Run from the absolutelib repo.
    exit /b 1
)
cd /d "%REPO_ROOT%"
echo [OK] Repo root: %REPO_ROOT%

:: ========================================================
:: SAFETY CHECK 2: Must be on master branch
:: ========================================================
for /f "tokens=*" %%b in ('git rev-parse --abbrev-ref HEAD') do set "CURRENT_BRANCH=%%b"
if /i not "%CURRENT_BRANCH%"=="master" (
    echo [ERROR] You must be on the 'master' branch to release. Current branch: %CURRENT_BRANCH%
    echo         Run: git checkout master
    exit /b 1
)
echo [OK] On branch: %CURRENT_BRANCH%

:: ========================================================
:: SAFETY CHECK 3: No uncommitted changes
:: ========================================================
git diff --quiet 2>nul
if errorlevel 1 (
    echo.
    echo [WARNING] You have unstaged changes:
    git diff --stat
    echo.
    set /p CONTINUE_DIRTY="Continue anyway? (y/n): "
    if /i not "!CONTINUE_DIRTY!"=="y" (
        echo Aborted.
        exit /b 1
    )
)
git diff --cached --quiet 2>nul
if errorlevel 1 (
    echo.
    echo [WARNING] You have staged but uncommitted changes:
    git diff --cached --stat
    echo.
    set /p CONTINUE_STAGED="Continue anyway? (y/n): "
    if /i not "!CONTINUE_STAGED!"=="y" (
        echo Aborted.
        exit /b 1
    )
)

:: ========================================================
:: SAFETY CHECK 4: Required files exist
:: ========================================================
set "MISSING="
if not exist "gradlew.bat"       set "MISSING=!MISSING! gradlew.bat"
if not exist "gradle.properties"  set "MISSING=!MISSING! gradle.properties"
if not exist "absolutelib.json"   set "MISSING=!MISSING! absolutelib.json"
if not exist "site\index.html"    set "MISSING=!MISSING! site\index.html"
if not exist "site\styles.css"    set "MISSING=!MISSING! site\styles.css"

if not "!MISSING!"=="" (
    echo [ERROR] Missing required files:%MISSING%
    exit /b 1
)
echo [OK] All required files present.

:: ========================================================
:: READ CURRENT VERSION
:: ========================================================
for /f "tokens=1,2 delims==" %%A in ('findstr /B "version=" gradle.properties') do set "OLD_VERSION=%%B"
if "%OLD_VERSION%"=="" set "OLD_VERSION=unknown"
echo.
echo Current version: %OLD_VERSION%

:: ========================================================
:: PROMPT FOR NEW VERSION
:: ========================================================
echo.
set /p NEW_VERSION="Enter new version (format x.x.x): "

:: Validate version format (must be x.x.x where x is a number)
powershell -Command "if ($env:NEW_VERSION -notmatch '^\d+\.\d+\.\d+$') { exit 1 } else { exit 0 }"
if errorlevel 1 (
    echo [ERROR] Invalid version format. Expected something like 2.1.0
    exit /b 1
)

if "!NEW_VERSION!"=="!OLD_VERSION!" (
    echo [WARNING] New version is the same as the current version ^(!OLD_VERSION!^).
    set /p SAME_VER="Continue with the same version? (y/n): "
    if /i not "!SAME_VER!"=="y" (
        echo Aborted.
        exit /b 1
    )
)

echo.
echo  Releasing: !OLD_VERSION! --^> !NEW_VERSION!
echo.

:: ========================================================
:: PROMPT FOR COMMIT MESSAGE
:: ========================================================
set /p COMMIT_MSG="Enter commit message (or press Enter for default): "
if "!COMMIT_MSG!"=="" set "COMMIT_MSG=Release v!NEW_VERSION!"

echo.
echo  Version : !NEW_VERSION!
echo  Message : !COMMIT_MSG!
echo.
set /p CONFIRM="Proceed with release? (y/n): "
if /i not "!CONFIRM!"=="y" (
    echo Aborted.
    exit /b 1
)

:: ========================================================
:: STEP 1: UPDATE gradle.properties
:: ========================================================
echo.
echo [1/7] Updating gradle.properties ...
powershell -Command "(Get-Content 'gradle.properties') -replace '^version=.*', ('version=' + $env:NEW_VERSION) | Set-Content 'gradle.properties'"
echo       version=!NEW_VERSION!

:: ========================================================
:: STEP 2: UPDATE absolutelib.json (vendor JSON)
:: ========================================================
echo [2/7] Updating absolutelib.json ...
set "JSON_FILE=absolutelib.json"

powershell -Command " $json = Get-Content $env:JSON_FILE | ConvertFrom-Json; $json.version = $env:NEW_VERSION; if ($json.javaDependencies.Count -gt 0) { $json.javaDependencies[0].version = $env:NEW_VERSION }; $json.mavenUrls = @('https://team4308.github.io/absolutelib/lib', 'https://jitpack.io'); $json.jsonUrl = 'https://team4308.github.io/absolutelib/lib/absolutelib.json'; $json | ConvertTo-Json -Depth 10 | Set-Content $env:JSON_FILE "

:: Copy to site/lib
if not exist "site\lib" mkdir "site\lib"
copy /Y "%JSON_FILE%" "site\lib\absolutelib.json" >nul
echo       Vendor JSON updated and copied to site\lib.

:: ========================================================
:: STEP 3: CREATE LOCAL BACKUP
:: ========================================================
echo [3/7] Creating local backup ...
set "BACKUP_DIR=.git-backup-cache"
if not exist "%BACKUP_DIR%" mkdir "%BACKUP_DIR%"

for /f "tokens=1-4 delims=/: " %%a in ("%date%") do (
    for /f "tokens=1-3 delims=:." %%i in ("%time%") do (
        set "TS=%%d-%%b-%%c-%%i-%%j-%%k"
    )
)
set "CACHE_PATH=!BACKUP_DIR!\!TS!-v!NEW_VERSION!"
mkdir "%CACHE_PATH%" 2>nul

if exist ".git-exclude-list.txt" (
    xcopy * "%CACHE_PATH%\" /E /I /H /Y /EXCLUDE:.git-exclude-list.txt >nul
) else (
    xcopy * "%CACHE_PATH%\" /E /I /H /Y >nul
)
echo       Backup saved to %CACHE_PATH%

:: ========================================================
:: STEP 4: COMMIT & PUSH MASTER
:: ========================================================
echo [4/7] Committing and pushing master ...
git add -A
git restore --staged site >nul 2>&1
git commit -m "!COMMIT_MSG!"
if errorlevel 1 (
    echo [WARNING] Nothing to commit on master, or commit failed.
)
git push origin master
if errorlevel 1 (
    echo [ERROR] Failed to push to master.
    echo         Check your permissions and remote URL.
    exit /b 1
)
echo       Master branch pushed.

:: ========================================================
:: STEP 5: BUILD WITH GRADLE (Javadoc + Maven pages repo)
:: ========================================================
echo [5/7] Running Gradle build ...
call .\gradlew.bat "-PpagesPublish=true" publishPagesRepo --no-daemon
if errorlevel 1 (
    echo [ERROR] Gradle publishPagesRepo failed. Aborting.
    exit /b 1
)
echo       Gradle build complete.

:: ========================================================
:: STEP 6: STAGE THE SITE
:: ========================================================
echo [6/7] Staging site for deployment ...
set "OUT_DIR=build\pages-site"
rmdir /S /Q "%OUT_DIR%" 2>nul
mkdir "%OUT_DIR%" 2>nul

:: 6a) Copy static site source
xcopy "site\*" "%OUT_DIR%\" /E /I /Y >nul
if errorlevel 1 (
    echo [ERROR] Failed copying site\ to %OUT_DIR%.
    exit /b 1
)

:: 6b) Copy Maven repo output
if exist "build\pages-maven" (
    mkdir "%OUT_DIR%\lib" 2>nul
    xcopy /E /I /Y "build\pages-maven\*" "%OUT_DIR%\lib\" >nul
) else (
    echo       [WARN] build\pages-maven not found. Skipping.
)

:: 6c) Copy Javadocs
if exist "build\docs\javadoc" (
    mkdir "%OUT_DIR%\docs\javadoc" 2>nul
    xcopy /E /I /Y "build\docs\javadoc\*" "%OUT_DIR%\docs\javadoc\" >nul
) else (
    echo       [WARN] build\docs\javadoc not found. Skipping.
)

:: 6d) Apply custom Javadoc dark theme
if exist "site\javadoc-theme.css" (
    if exist "%OUT_DIR%\docs\javadoc" (
        echo       Applying custom Javadoc dark theme...
        copy /Y "site\javadoc-theme.css" "%OUT_DIR%\docs\javadoc\stylesheet.css" >nul
    )
)

:: 6e) Copy vendor JSON into lib
mkdir "%OUT_DIR%\lib" 2>nul
copy /Y "absolutelib.json" "%OUT_DIR%\lib\absolutelib.json" >nul

echo       Site staged at %OUT_DIR%

:: ========================================================
:: STEP 7: DEPLOY TO gh-pages
:: ========================================================
echo [7/7] Deploying to gh-pages branch ...
set "REMOTE_URL=https://github.com/Team4308/absolutelib.git"
set "TARGET_BRANCH=gh-pages"

pushd "%OUT_DIR%"
git init
git add -A
for /f "tokens=*" %%D in ('date /t') do set "BUILD_DATE=%%D"
git commit -m "Deploy v!NEW_VERSION! - !BUILD_DATE!"
git branch -M %TARGET_BRANCH%
git push "%REMOTE_URL%" %TARGET_BRANCH% --force
if errorlevel 1 (
    popd
    echo [ERROR] Failed to push to %TARGET_BRANCH%.
    echo         Check your permissions and the remote URL.
    exit /b 1
)
popd
echo       gh-pages deployed.

:: ========================================================
:: DONE
:: ========================================================
echo.
echo  =============================================
echo   Release v!NEW_VERSION! complete!
echo  =============================================
echo.
echo   Master branch:  pushed
echo   gh-pages:       deployed
echo   Backup:         !CACHE_PATH!
echo   Vendor JSON:    updated to v!NEW_VERSION!
echo.
pause
