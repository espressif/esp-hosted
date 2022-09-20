@echo off
rem ### This is windows platform script to copy the project files and 
rem ### make them aligned to STM32CubeIDE needs

set WORKSPACE=%1
set PROJ_NAME=stm_spi_host
set CWD=%CD%

rem ### Check argument passed ###
IF "%1"=="" (
echo usage: %0 Workspace_directory_absolute_path
goto error
)

rem ### store git repo base path###
cd ..\..\..
set CODE_BASE=%CD%
cd %CWD%

rem  ### check workspace directory exist ###
IF not exist %WORKSPACE% (
echo %WORKSPACE% does not exist
echo Please follow documentation to import STM project from stm_spi_host_<ESP_slave_board_type>.ioc, if not already done
goto error
 )

rem  ### check project directory exist ###
IF not exist %WORKSPACE%\%PROJ_NAME%  (
echo %WORKSPACE%\%PROJ_NAME% does not exist
echo Either incorrect workspace directory or ioc project not imported
echo Please follow documentation to import STM project from stm_spi_host_<ESP_slave_board_type>.ioc, if not already done
goto error
 )

rem ### search and replace project files ###
DEL %WORKSPACE%\%PROJ_NAME%\.project 2>NUL
DEL %WORKSPACE%\%PROJ_NAME%\.cproject 2>NUL

setLocal EnableDelayedExpansion
For /f "tokens=* delims= " %%a in (.project) do (
Set str=%%a
set str=!str:CODE_BASE_PLACE_HOLDER=%CODE_BASE%!
echo !str!>>  %WORKSPACE%\%PROJ_NAME%\.project
)
ENDLOCAL


setLocal EnableDelayedExpansion
For /f "tokens=* delims= " %%a in (.cproject) do (
Set str=%%a
set str=!str:CODE_BASE_PLACE_HOLDER=%CODE_BASE%!
echo !str!>>  %WORKSPACE%\%PROJ_NAME%\.cproject
)
ENDLOCAL

rem ### touch .mxproject file ###
copy /b %WORKSPACE%\%PROJ_NAME%\.mxproject +,, >NUL

echo success. Now open STM32CubeIDE with %WORKSPACE%
:error
EXIT /B 1
