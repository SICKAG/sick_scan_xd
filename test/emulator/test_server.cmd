if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64
set PATH=%PYTHON_DIR%;%PYTHON_DIR%\DLLs;%PYTHON_DIR%\Lib;%PYTHON_DIR%\Scripts;%PATH%

python --version
python %*
