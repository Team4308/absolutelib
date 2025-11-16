@echo off
setlocal enabledelayedexpansion

REM Ensure git is available
where git >nul 2>nul
if errorlevel 1 (
  echo git not found in PATH
  echo Add Git to PATH or run from a Developer Command Prompt.
  exit /b 1
)

REM Ensure gradlew is available
if not exist ".\gradlew.bat" (
  echo gradlew.bat not found in current directory. Run this from C:\FRC\absolutelib
  exit /b 1
)

echo ================================
echo Publish to GitHub Packages
echo ================================
set /p GPR_USER_IN=Enter your GitHub username: 
set /p GPR_TOKEN_IN=Enter your GitHub PAT (scope: read:packages, write:packages): 

if "%GPR_USER_IN%"=="" (
  echo No username provided. Aborting.
  exit /b 1
)
if "%GPR_TOKEN_IN%"=="" (
  echo No token provided. Aborting.
  exit /b 1
)

echo.
echo Using user "%GPR_USER_IN%".
echo.

REM Export for Gradle to read
set GPR_USER=%GPR_USER_IN%
set GPR_TOKEN=%GPR_TOKEN_IN%

echo Running: gradlew verifyPublishCreds
call .\gradlew.bat verifyPublishCreds
if errorlevel 1 (
  echo verifyPublishCreds failed. Check credentials and try again.
  exit /b 1
)

echo.
echo Running: gradlew publish
call .\gradlew.bat publish
set ERR=%ERRORLEVEL%

if not "%ERR%"=="0" (
  echo.
  echo gradlew publish failed with exit code %ERR%.
  echo If the error mentions "status code 409", the artifact version is already published.
  echo To publish again, bump the version in gradle.properties.
  exit /b %ERR%
)

echo.
echo Publish completed successfully.
endlocal
