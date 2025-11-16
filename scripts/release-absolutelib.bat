@echo off
setlocal enabledelayedexpansion

REM AbsoluteLib release helper.
REM Steps:
REM  1. Ask for new version (e.g., 1.0.3) and a short note.
REM  2. Update:
REM       - gradle.properties (version)
REM       - absolutelib.json (version + javaDependencies)
REM       - README.md (version strings)
REM  3. Show git diff, confirm, commit, tag, push.
REM  4. Optionally run publish-github-packages.bat.

if not exist ".git" (
  echo This script must be run from the repo root: C:\FRC\absolutelib
  exit /b 1
)

echo ==========================================
echo AbsoluteLib Release Helper
echo ==========================================

REM Read current version from gradle.properties
for /f "tokens=1,2 delims==" %%A in ('findstr /B "version=" gradle.properties') do set CURRENT_VERSION=%%B

echo Current version in gradle.properties: %CURRENT_VERSION%
set /p NEW_VERSION=Enter new version (e.g. 1.0.3): 

if "%NEW_VERSION%"=="" (
  echo No version entered. Aborting.
  exit /b 1
)

echo New version: %NEW_VERSION%
set /p REL_NOTES=Enter short release note (one line, optional): 

echo.
echo Updating versions...

REM 1) gradle.properties: version line
powershell -Command "(Get-Content gradle.properties) -replace 'version=.*', 'version=%NEW_VERSION%' | Set-Content gradle.properties"

REM 2) absolutelib.json: top-level version and javaDependencies version
powershell -Command "(Get-Content absolutelib.json) -replace '\"version\": \"[0-9A-Za-z._-]+\"', '\"version\": \"%NEW_VERSION%\"' | Set-Content absolutelib.json"
powershell -Command "(Get-Content absolutelib.json) -replace '\"artifactId\": \"absolutelib-java\",\s*\"version\": \"[0-9A-Za-z._-]+\"', '\"artifactId\": \"absolutelib-java\",\n            \"version\": \"%NEW_VERSION%\"' | Set-Content absolutelib.json"

REM 3) README.md: update all visible version strings for Java and JitPack
powershell -Command "(Get-Content README.md) -replace 'absolutelib-java:[0-9A-Za-z._-]+', 'absolutelib-java:%NEW_VERSION%' | Set-Content README.md"
powershell -Command "(Get-Content README.md) -replace 'com.github.Team4308:absolutelib:[0-9A-Za-z._-]+', 'com.github.Team4308:absolutelib:%NEW_VERSION%' | Set-Content README.md"

echo.
echo Git status and diff:
git status
git diff

set /p CONFIRM=Proceed with commit and tag for %NEW_VERSION%? (y/N): 
if /I not "%CONFIRM%"=="y" (
  echo Aborting before commit.
  exit /b 0
)

set COMMIT_MSG=chore: release %NEW_VERSION%
if not "%REL_NOTES%"=="" (
  set COMMIT_MSG=chore: release %NEW_VERSION% - %REL_NOTES%
)

echo.
echo Committing changes...
REM Add all modified and new files so other edits (pages.yml, build.gradle, this script)
REM are included in the release commit.
git add -A
git commit -m "%COMMIT_MSG%"
if errorlevel 1 (
  echo git commit failed. Check output above.
  exit /b 1
)

echo.
echo Creating tag v%NEW_VERSION%...
git tag v%NEW_VERSION%
if errorlevel 1 (
  echo git tag failed. You may already have this tag.
  exit /b 1
)

echo.
echo Pushing commit and tag to origin...
git push
set PUSH_ERR=%ERRORLEVEL%
git push origin v%NEW_VERSION%
if errorlevel 1 (
  set PUSH_ERR=1
)

if not "%PUSH_ERR%"=="0" (
  echo.
  echo git push failed (branch may be out of date or diverged).
  set /p FORCE_PUSH=Force push to origin (git push --force-with-lease)? (y/N): 
  if /I "%FORCE_PUSH%"=="y" (
    echo.
    echo Running git push --force-with-lease ...
    git push --force-with-lease
    if errorlevel 1 (
      echo Force push failed. Resolve git issues manually.
      exit /b 1
    )
    echo.
    echo Pushing tag v%NEW_VERSION%...
    git push origin v%NEW_VERSION%
    if errorlevel 1 (
      echo Tag push failed after force push. Resolve manually.
      exit /b 1
    )
  ) else (
    echo Skipping force push. Resolve git state manually.
    exit /b 1
  )
)

echo.
echo Release %NEW_VERSION% pushed.
echo This will trigger:
echo   - update-vendor-json.yml on tag v%NEW_VERSION%
echo   - pages.yml on main/tag to publish Maven repo to GitHub Pages.

set /p PUBLISH_PKGS=Also run scripts\publish-github-packages.bat now? (y/N): 
if /I "%PUBLISH_PKGS%"=="y" (
  if exist ".\scripts\publish-github-packages.bat" (
    echo.
    echo Running scripts\publish-github-packages.bat ...
    call .\scripts\publish-github-packages.bat
  ) else (
    echo scripts\publish-github-packages.bat not found.
  )
)

echo.
echo Release helper finished.
endlocal
