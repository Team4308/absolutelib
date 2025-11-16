@echo off
setlocal enabledelayedexpansion

echo Publishing Static Site to GitHub Pages

for /f "tokens=2" %%b in ('git symbolic-ref --short HEAD') do set CURRENT_BRANCH=%%b

set "COMMIT_MSG="
set /p COMMIT_MSG=Enter commit message: 

if "%COMMIT_MSG%"=="" (
    echo No commit message entered. Aborting.
    goto :eof
)

git checkout -B gh-pages
git rm -rf .


xcopy site\* . /E /I /Y

git add .
git commit -m "%COMMIT_MSG%"
git push origin gh-pages --force

git checkout "%CURRENT_BRANCH%"

endlocal