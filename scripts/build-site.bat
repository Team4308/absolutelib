@echo off
setlocal enabledelayedexpansion

REM Build static Maven repo and site content into .\site for GitHub Pages.
REM Usage:
REM   scripts\build-site.bat

if not exist ".git" (
  echo Run this from the repo root: C:\FRC\absolutelib
  exit /b 1
)

echo ==========================================
echo Build-site: Generating GitHub Pages content
echo ==========================================

REM Ensure gradlew exists
if not exist ".\gradlew.bat" (
  echo gradlew.bat not found. Run from repo root.
  exit /b 1
)

REM Use version from gradle.properties
for /f "tokens=1,2 delims==" %%A in ('findstr /B "version=" gradle.properties') do set CURRENT_VERSION=%%B
echo Using version: %CURRENT_VERSION%

echo.
echo Running: gradlew -PpagesPublish=true publishPagesRepo
call .\gradlew.bat "-PpagesPublish=true" publishPagesRepo --no-daemon
if errorlevel 1 (
  echo Gradle publishPagesRepo failed. Aborting.
  exit /b 1
)

echo.
echo Preparing ./site directory...
rmdir /S /Q site 2>nul
mkdir site

REM Copy Maven repo to site root
xcopy /E /I /Y build\pages-maven site\ >nul

REM Copy vendor JSON to site root
copy /Y absolutelib.json site\absolutelib.json >nul

echo.
echo Site content ready under .\site
echo Next step: scripts\publish-latest.bat
echo.
endlocal
