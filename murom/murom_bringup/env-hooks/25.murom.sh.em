# Set some sane defaults for the turtlebot launch environment

##Documentation: 
#  The colon command simply has its arguments evaluated and then succeeds. 
#   It is the original shell comment notation (before '#' to end of line). For a long time, Bourne shell scripts had a colon as the first character. 
#   The C Shell would read a script and use the first character to determine whether it was for the C Shell (a '#' hash) or the Bourne shell (a ':' colon).
#   Then the kernel got in on the act and added support for '#!/path/to/program' and the Bourne shell got '#' comments, and the colon convention went by the wayside. 
#   But if you come across a script that starts with a colon (Like this one), now you will know why. ~ Jonathan Leffler

: ${MUROM_BASE:=murom}                           # create, roomba
: ${MUROM_BATTERY:=/sys/class/power_supply/BAT0}  # /proc/acpi/battery/BAT0 in 2.6 or earlier kernels,  /sys/class/power_supply/ (kernels 3.0+) 
: ${MUROM_STACKS:=hexagons}                       # circles, hexagons
: ${MUROM_3D_SENSOR:=kinect}              # kinect, asus_xtion_pro, asus_xtion_pro_offset
: ${MUROM_SIMULATION:=false}
: ${MUROM_SERIAL_PORT:=/dev/murom}               # /dev/ttyUSB0, /dev/ttyS0

: ${MUROM_NAME:=murom}
: ${MUROM_TYPE:=murom}
: ${MUROM_RAPP_PACKAGE_WHITELIST:=[rocon_apps, murom_rapps]}
: ${MUROM_RAPP_PACKAGE_BLACKLIST:=[]}
: ${MUROM_INTERACTIONS_LIST:=[murom_bringup/admin.interactions, murom_bringup/documentation.interactions, murom_bringup/pairing.interactions, murom_bringup/visualisation.interactions]}

# Exports
export MUROM_BASE
export MUROM_BATTERY
export MUROM_STACKS
export MUROM_3D_SENSOR
export MUROM_SIMULATION
export MUROM_SERIAL_PORT
export MUROM_NAME
export MUROM_TYPE
export MUROM_RAPP_PACKAGE_WHITELIST
export MUROM_RAPP_PACKAGE_BLACKLIST
export MUROM_INTERACTIONS_LIST
