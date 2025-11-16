@echo off

REM Publish-Latest: commit/push latest code and update gh-pages from ./site.

if not exist ".git" (
  echo Run this from the repo root: C:\FRC\absolutelib
  exit /b 1
)

echo ==========================================
echo Publish-Latest: Push latest code and Maven repo
echo ==========================================

REM Read current version from gradle.properties (no delayed expansion)
set CURRENT_VERSION=
for /f "tokens=1,* delims==" %%A in ('findstr /B "version=" gradle.properties') do set CURRENT_VERSION=%%B
echo Current version: %CURRENT_VERSION%

echo.
set RUN_BUILD_SITE=N
set /p RUN_BUILD_SITE=Run Build-site (gradlew -PpagesPublish publishPagesRepo)? (y/N): 
if /I "%RUN_BUILD_SITE%"=="Y" (
  call scripts\build-site.bat
  if errorlevel 1 (
    echo Build-site failed. Aborting.
    exit /b 1
  )
)

echo.
echo Git status before commit:
git status

echo.
set DO_COMMIT=N
set /p DO_COMMIT=Commit and push current branch (code only)? (y/N): 
if /I "%DO_COMMIT%"=="Y" (
  set COMMIT_MSG=
  set /p COMMIT_MSG=Enter commit message [default: Publish latest %CURRENT_VERSION%]: 
  if "%COMMIT_MSG%"=="" set COMMIT_MSG=Publish latest %CURRENT_VERSION%
  echo.
  echo Committing code changes...
  git add .
  git commit -m "%COMMIT_MSG%"
  if errorlevel 1 (
    echo git commit failed (maybe no changes). Continuing.
  ) else (
    echo.
    echo Pushing current branch...
    git push
    if errorlevel 1 (
      echo git push failed. Fix and rerun if needed.
    )
  )
) else (
  echo Skipping code commit/push.
)

echo.
set UPDATE_PAGES=N
set /p UPDATE_PAGES=Also update gh-pages branch from ./site? (y/N): 
if /I not "%UPDATE_PAGES%"=="Y" (
  echo Skipping gh-pages update.
  goto done
)

if not exist "site" (
  echo ./site not found, running Build-site first...
  call scripts\build-site.bat
  if errorlevel 1 (
    echo Build-site failed. Aborting gh-pages update.
    goto done
  )
)

echo.
echo Ensuring gh-pages branch exists locally or on origin...
git fetch origin gh-pages 2>nul

git rev-parse --verify gh-pages >nul 2>&1
if errorlevel 1 (
  git show-ref --verify --quiet refs/remotes/origin/gh-pages
  if errorlevel 1 (
    echo Creating new orphan gh-pages branch...
    git checkout --orphan gh-pages
    git rm -rf . >nul 2>&1
  ) else (
    echo Creating local gh-pages tracking origin/gh-pages...
    git checkout -b gh-pages origin/gh-pages
  )
) else (
  echo Switching to existing gh-pages branch...
  git checkout gh-pages
)

echo Clearing gh-pages contents...
git rm -rf . >nul 2>&1

echo Copying ./site into gh-pages...
xcopy /E /I /Y site\ . >nul 2>&1

echo.
git add .
git commit -m "Update Pages site for %CURRENT_VERSION%" 2>nul
if errorlevel 1 (
  echo git commit on gh-pages failed (maybe no changes). Continuing.
) else (
  echo.
  echo Pushing gh-pages...
  git push -u origin gh-pages
  if errorlevel 1 (
    echo git push gh-pages failed. Check your remote and try again.
  )
)

echo.
echo Switching back to previous branch...
git checkout - >nul 2>&1

:done
echo.
echo Publish-Latest finished.
echo Check:
echo   https://team4308.github.io/absolutelib/absolutelib.json
echo   https://team4308.github.io/absolutelib/ca/team4308/absolutelib-java/%CURRENT_VERSION%/absolutelib-java-%CURRENT_VERSION%.pom
