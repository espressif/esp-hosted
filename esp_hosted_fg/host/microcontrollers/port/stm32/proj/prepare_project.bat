@echo off
rem ### This is windows platform script to copy the project files and
rem ### make them aligned to STM32CubeIDE needs

set TRANSPORT=%1
set WORKSPACE=%2
set PROJ_NAME=stm_%TRANSPORT%_host
set CWD=%CD%

rem ### Check incorrect number of argument passed
set argC=0
for %%x in (%*) do Set /A argC+=1

if %argC% gtr 2 (
echo Invalid number of arguments entered
goto error
)

rem ### Check argument passed ###
if (%1% == "" or %2% == "") (
echo usage: %0 Transport Workspace_directory_absolute_path
echo Transport - either spi or sdio
echo Workspace_directory_absolute_path - Workspace directory created for STM32CubeIDE
goto error
)

rem ### store git repo base path###
cd ..\..\..
set CODE_BASE=%CD%
echo code base %CD%
cd %CWD%

rem  ### check workspace directory exist ###
IF not exist %WORKSPACE% (
echo %WORKSPACE% does not exist
echo For <SPI> transport and ESP one from <ESP32-C2/C3/C6/S2/S3> use spi/stm_spi_host_v2.ioc, if not already done
echo For <SPI> transport and <ESP32> use spi/stm_spi_host_v1.ioc, if not already done
echo For <SDIO> transport and <ESP32> use sdio/stm_sdio_host.ioc, if not already done

goto error
 )

rem  ### check project directory exist ###
IF not exist %WORKSPACE%\%PROJ_NAME%  (
echo %WORKSPACE%\%PROJ_NAME% does not exist
echo Either incorrect workspace directory or ioc project not imported
echo For <SPI> transport and ESP one from <ESP32-C2/C3/C6/S2/S3> use spi/stm_spi_host_v2.ioc, if not already done
echo For <SPI> transport and <ESP32> use spi/stm_spi_host_v1.ioc, if not already done
echo For <SDIO> transport and <ESP32> use sdio/stm_sdio_host.ioc, if not already done
goto error
 )

rem ### search and replace project files ###
DEL %WORKSPACE%\%PROJ_NAME%\.project 2>NUL
DEL %WORKSPACE%\%PROJ_NAME%\.cproject 2>NUL

rem ### .project file requires '/' instead of '\' as filepath separator
rem ### and leading '/' before the filepath to work properly
rem ### in STM32CubeIDE on Windows 11
rem ### example: c:\esp\esp_hosted\esp_hosted_fg is converted to:
rem ###          /c:/esp/esp_hosted/esp_hosted_fg
set "CODE_BASE_2=/%CODE_BASE:\=/%"

setLocal EnableDelayedExpansion
For /f "tokens=* delims= " %%a in (%TRANSPORT%\.project) do (
Set str=%%a
set str=!str:CODE_BASE_PLACE_HOLDER=%CODE_BASE_2%!
echo !str!>>  %WORKSPACE%\%PROJ_NAME%\.project
)
ENDLOCAL


setLocal EnableDelayedExpansion
For /f "tokens=* delims= " %%a in (%TRANSPORT%\.cproject) do (
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
