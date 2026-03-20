Connects tar1090 + readadsb to rotctl for use with an antenna rotator like Discovery Drive. Allows the antenna rotator to automatically track aircraft with a camera.

Required a tar1090 + readasb installation to already be set up. We recommend using the script at https://github.com/wiedehopf/adsb-scripts/wiki/Automatic-installation-for-readsb which automatically installs both readsb and tar1090.

First see for information on how to set up the TAR1090 hook in TAR1090_HOOK_SETUP.txt, 
so that you can click to point the rotator.

How to run:

python3 adsb_rotctl.py ROTATOR_IP:ROTATOR_PORT

Default rotator port: 4533
