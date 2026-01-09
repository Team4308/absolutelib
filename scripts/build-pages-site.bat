@echo off
setlocal enabledelayedexpansion


set "REMOTE_URL=https://github.com/Team4308/absolutelib.git"
set "TARGET_BRANCH=gh-pages"

REM Must be run from repo root (or anywhere inside repo)
for /f "delims=" %%R in ('git rev-parse --show-toplevel 2^>nul') do set "REPO_ROOT=%%R"
if "%REPO_ROOT%"=="" goto no_repo
goto has_repo
:no_repo
echo ERROR: Not inside a git repository. Run from the absolutelib repo.
exit /b 1
:has_repo
cd /d "%REPO_ROOT%"

REM Validate source docs exist
if not exist "site\index.html" goto no_index
goto has_index
:no_index
echo ERROR: site\index.html not found. You deleted the source docs.
exit /b 1
:has_index
if not exist "site\styles.css" goto no_styles
goto has_styles
:no_styles
echo ERROR: site\styles.css not found. You deleted the source docs.
exit /b 1
:has_styles

echo ==========================================
echo Building static site for GitHub Pages
echo Repo: %REPO_ROOT%
echo ==========================================

REM Ensure gradlew exists
if not exist ".\gradlew.bat" goto no_gradlew
goto has_gradlew
:no_gradlew
echo ERROR: gradlew.bat not found. Run from repo root.
exit /b 1
:has_gradlew

REM Use version from gradle.properties (optional)
for /f "tokens=1,2 delims==" %%A in ('findstr /B "version=" gradle.properties 2^>nul') do set "CURRENT_VERSION=%%B"
if "%CURRENT_VERSION%"=="" set "CURRENT_VERSION=unknown"
echo Using version: %CURRENT_VERSION%

echo.
echo Running: gradlew -PpagesPublish=true publishPagesRepo
call .\gradlew.bat "-PpagesPublish=true" publishPagesRepo --no-daemon
if errorlevel 1 goto gradle_failed
goto gradle_success
:gradle_failed
echo ERROR: Gradle publishPagesRepo failed. Aborting.
exit /b 1
:gradle_success

REM Stage folder (do NOT touch ./site)
set "OUT_DIR=build\pages-site"
echo.
echo Preparing staging folder: %OUT_DIR%
rmdir /S /Q "%OUT_DIR%" 2>nul
mkdir "%OUT_DIR%" 2>nul

REM 1) Copy static docs source (index + css + any other assets you add later)
xcopy "site\*" "%OUT_DIR%\" /E /I /Y >nul
if errorlevel 1 goto copy_failed
goto copy_success
:copy_failed
echo ERROR: Failed copying ./site to %OUT_DIR%.
exit /b 1
:copy_success

REM 2) Copy Maven repo output
if not exist "build\pages-maven" goto skip_maven
mkdir "%OUT_DIR%\lib" 2>nul
xcopy /E /I /Y "build\pages-maven\*" "%OUT_DIR%\lib\" >nul
goto done_maven
:skip_maven
echo WARN: build\pages-maven not found (Gradle task may not have produced it).
:done_maven

REM 3) Copy Javadocs (optional)
if not exist "build\docs\javadoc" goto skip_javadoc
mkdir "%OUT_DIR%\docs\javadoc" 2>nul
xcopy /E /I /Y "build\docs\javadoc\*" "%OUT_DIR%\docs\javadoc\" >nul
goto done_javadoc
:skip_javadoc
echo WARN: build\docs\javadoc not found.
:done_javadoc

REM 4) Apply custom dark theme to Javadoc
set "APPLY_THEME=0"
if exist "site\javadoc-theme.css" set "APPLY_THEME=1"
if not exist "%OUT_DIR%\docs\javadoc" set "APPLY_THEME=0"
if not "%APPLY_THEME%"=="1" goto skip_theme
echo Applying custom Javadoc dark theme...
copy /Y "site\javadoc-theme.css" "%OUT_DIR%\docs\javadoc\stylesheet.css" >nul
:skip_theme

REM 5) Copy vendordep JSON into lib
mkdir "%OUT_DIR%\lib" 2>nul
copy /Y "absolutelib.json" "%OUT_DIR%\lib\absolutelib.json" >nul

echo.
echo Staged site contents into: %OUT_DIR%

REM Make sure repo is clean enough to run subtree (subtree uses git history)
REM (We won't hard-fail, just warn.)
git diff --quiet
if errorlevel 1 echo WARN: You have uncommitted changes. Will commit them before pushing.

echo.
echo Adding all changes...
git add -A

echo.
echo Committing changes...
for /f "tokens=*" %%D in ('date /t') do set "BUILD_DATE=%%D"
git commit -m "Build site - %BUILD_DATE% - v%CURRENT_VERSION%" --allow-empty

echo.
echo Pushing to %TARGET_BRANCH% branch at %REMOTE_URL%...
git push "%REMOTE_URL%" HEAD:%TARGET_BRANCH%
if errorlevel 1 goto push_failed
goto push_success
:push_failed
echo ERROR: Failed to push to %TARGET_BRANCH%.
echo        Make sure you have permission and the remote URL is correct.
exit /b 1
:push_success

echo.
echo Done. Published %CURRENT_VERSION% to %TARGET_BRANCH% branch.
exit /b 0