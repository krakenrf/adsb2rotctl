Connects tar1090 + readsb to rotctl for use with an antenna rotator like Discovery Drive. Allows the antenna rotator to automatically track aircraft with a camera.

Required a tar1090 + readasb installation to already be set up. We recommend using the script at https://github.com/wiedehopf/adsb-scripts/wiki/Automatic-installation-for-readsb which automatically installs both readsb and tar1090. 

Remember to set your --lat and --lon in readsb settings. sudo nano /etc/default/readsb

First see TAR1090_HOOK_SETUP.txt for information on how to set up the TAR1090 hook. This will enable you to select aircraft in the tar1090 webinterface by clicking on the desired aircraft to track. 

If no aircraft is selected, the default behaviour is to track the closest aircraft.

How to run:

python3 adsb_rotctl.py ROTATOR_IP:ROTATOR_PORT --alt ROTATOR_STATION_ALTITUDE_ABOVE_SEA_LEVEL_IN_METERS

Default rotator port: 4533
