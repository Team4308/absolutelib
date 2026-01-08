@echo off
setlocal enabledelayedexpansion

REM ==========================================
REM Build + publish docs site to gh-pages
REM - Does NOT delete ./site (source)
REM - Stages output in ./build/pages-site
REM - Publishes via git subtree
REM ==========================================

REM Must be run from repo root (or anywhere inside repo)
for /f "delims=" %%R in ('git rev-parse --show-toplevel 2^>nul') do set "REPO_ROOT=%%R"
if "%REPO_ROOT%"=="" (
  echo ERROR: Not inside a git repository. Run from the absolutelib repo.
  exit /b 1
)
cd /d "%REPO_ROOT%"

REM Validate source docs exist
if not exist "site\index.html" (
  echo ERROR: site\index.html not found. You deleted the source docs.
  exit /b 1
)
if not exist "site\styles.css" (
  echo ERROR: site\styles.css not found. You deleted the source docs.
  exit /b 1
)

echo ==========================================
echo Building static site for GitHub Pages
echo Repo: %REPO_ROOT%
echo ==========================================

REM Ensure gradlew exists
if not exist ".\gradlew.bat" (
  echo ERROR: gradlew.bat not found. Run from repo root.
  exit /b 1
)

REM Use version from gradle.properties (optional)
for /f "tokens=1,2 delims==" %%A in ('findstr /B "version=" gradle.properties 2^>nul') do set "CURRENT_VERSION=%%B"
if "%CURRENT_VERSION%"=="" set "CURRENT_VERSION=unknown"
echo Using version: %CURRENT_VERSION%

echo.
echo Running: gradlew -PpagesPublish=true publishPagesRepo
call .\gradlew.bat "-PpagesPublish=true" publishPagesRepo --no-daemon
if errorlevel 1 (
  echo ERROR: Gradle publishPagesRepo failed. Aborting.
  exit /b 1
)

REM Stage folder (do NOT touch ./site)
set "OUT_DIR=build\pages-site"
echo.
echo Preparing staging folder: %OUT_DIR%
rmdir /S /Q "%OUT_DIR%" 2>nul
mkdir "%OUT_DIR%" 2>nul

REM 1) Copy static docs source (index + css + any other assets you add later)
xcopy "site\*" "%OUT_DIR%\" /E /I /Y >nul
if errorlevel 1 (
  echo ERROR: Failed copying ./site to %OUT_DIR%.
  exit /b 1
)

REM 2) Copy Maven repo output
if exist "build\pages-maven" (
  mkdir "%OUT_DIR%\lib" 2>nul
  xcopy /E /I /Y "build\pages-maven\*" "%OUT_DIR%\lib\" >nul
) else (
  echo WARN: build\pages-maven not found (Gradle task may not have produced it).
)

REM 3) Copy Javadocs (optional)
if exist "build\docs\javadoc" (
  mkdir "%OUT_DIR%\docs\javadoc" 2>nul
  xcopy /E /I /Y "build\docs\javadoc\*" "%OUT_DIR%\docs\javadoc\" >nul
) else (
  echo WARN: build\docs\javadoc not found.
)

REM 4) Copy vendordep JSON into lib
mkdir "%OUT_DIR%\lib" 2>nul
copy /Y "absolutelib.json" "%OUT_DIR%\lib\absolutelib.json" >nul

echo.
echo Staged site contents into: %OUT_DIR%

REM Make sure repo is clean enough to run subtree (subtree uses git history)
REM (We won't hard-fail, just warn.)
git diff --quiet
if errorlevel 1 (
  echo WARN: You have uncommitted changes. Subtree push will still work, but be careful.
)

echo.
echo Publishing to gh-pages (git subtree push)...
git subtree push --prefix "%OUT_DIR%" origin gh-pages
if errorlevel 1 (
  echo ERROR: Failed to push subtree to gh-pages.
  echo        Make sure you have permission and the remote 'origin' is correct.
  exit /b 1
)

echo.
echo Done. Published %CURRENT_VERSION% to gh-pages.
exit /b 0