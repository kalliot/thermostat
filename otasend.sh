#!/bin/sh
cat sdkconfig | grep CONFIG_APP_PROJECT_VER > vernum.tmp
. ./vernum.tmp
echo $CONFIG_APP_PROJECT_VER
FNAME="thermostat_$CONFIG_APP_PROJECT_VER"
echo $FNAME
message='{"dev":"5bdddc","id":"otaupdate","file":'\"${FNAME}\"'}'
echo $message
sftp pi@192.168.101.233 << EOF
cd srv/ota
ut build/thermostat.bin $FNAME
EOF
echo '{"dev":"5bdddc","id":"otaupdate","file":${FNAME}}'
mosquitto_pub -h 192.168.101.231 -t home/kallio/thermostat/5bdddc/otaupdate -m $message
echo 'DONE'
