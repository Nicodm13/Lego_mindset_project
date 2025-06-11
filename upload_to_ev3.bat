@echo off
set EV3_IP=192.168.93.19
set USER=robot
set RELATIVE_SRC=robot
set DEST_PATH=/home/robot/

echo Uploading %RELATIVE_SRC% to %USER%@%EV3_IP%:%DEST_PATH%
scp -r %RELATIVE_SRC% %USER%@%EV3_IP%:%DEST_PATH%
pause
