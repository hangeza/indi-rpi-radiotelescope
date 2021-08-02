#!/bin/bash
# scan horizontal window in a grid and take measurement at each point
# 05.06.2010 HGZ

# nr of values to acquire for one point
AVERAGE=50
# Timeout for waiting until RT reached set position in seconds
TIMEOUT=300
#step size in degrees
STEP_AZ_DEFAULT=1.0
STEP_ALT_DEFAULT=1.0

cmd_stop='indi_setprop "Pi Radiotelescope.TELESCOPE_ABORT_MOTION.ABORT=On"'

prop_az='"Pi Radiotelescope.HORIZONTAL_EOD_COORD.AZ"'
prop_alt='"Pi Radiotelescope.HORIZONTAL_EOD_COORD.ALT"'

# function goto_pos()
# go to given coordinates and wait until reached there
# goto_pos az alt [timeout]
# az,alt...coordinate to go
# timeout...optional timeout after which the function returns 
goto_pos()
{
        _az=$1
        _alt=$2

        if [[ $(echo "scale=6; $1 < 0.0"|bc) -eq 1 ]]; then
                _az=$(echo "scale=6; $1+360.0" | bc)
        elif [[ $(echo "scale=6; $1 >= 360.0"|bc) -eq 1 ]]; then
                _az=$(echo "scale=6; $1-360.0" | bc)
        fi

        tout="$TIMEOUT"
        if [ "$3" ] # Is parameter #3 !zero length?
   then
                tout="$3"
   fi

        # goto position
        echo -n $(indi_setprop "Pi Radiotelescope.HORIZONTAL_EOD_COORD.AZ;ALT=$_az;$_alt")
   sleep 1
   indi_eval -w '"Pi Radiotelescope.SCOPE_STATUS.SCOPE_IDLE"==1'
}



if [ $# -lt 5 ]; then
  echo "Scan Horizontal Window in a grid and take measurement at each point"
  echo "need at least 5 arguments (horizontal coordinate boundaries and filename) for scan"
  echo "usage: $0 <min_az> <max_az> <min_alt> <max_alt> <file> [<step_az> <step_alt> <avg>]"
  echo " parameters:"
  echo "  min_az, min_alt - coordinate of lower left corner"
  echo "  max_az, max_alt - coordinate of upper right corner"
  echo "  file - output file name"
  echo "  step_az, step_alt - (optional) step size of coordinates in degree (default 1)"
  echo "  avg - (optional) take average of avg samples at each point (default 100)"
  exit 1
fi

if [[ $(echo "scale=6; $3 > $4"|bc) -eq 1 ]]; then
  echo "error: min alt > max alt"
  echo "exiting"
  exit 1
fi

if [[ $(echo "scale=6; $3 < -2.5"|bc) -eq 1 ]]; then
  echo "error: min alt below horizon"
  echo "exiting"
  exit 1
fi

if [[ $(echo "scale=6; $4 > 90.0"|bc) -eq 1 ]]; then
  echo "error: max alt exceeding zenith"
  echo "exiting"
  exit 1
fi

if [[ $(echo "scale=6; $1 > $2"|bc) -eq 1 ]]; then
  echo "error: min az > max az"
  echo "exiting"
  exit 1
fi

STEP_AZ=$STEP_AZ_DEFAULT
STEP_ALT=$STEP_ALT_DEFAULT

if [ $# -gt 5 ]; then
        STEP_AZ=$6
fi

if [ $# -gt 6 ]; then
        STEP_ALT=$7
fi

if [ $# -gt 7 ]; then
        AVERAGE=$8
fi

min_az=$1
max_az=$2
min_alt=$3
max_alt=$4

echo $(eval $cmd_stop)

echo "southern boundary = " $min_alt
echo "northern boundary = " $max_alt
echo "eastern boundary = " $min_az
echo "western boundary = " $max_az
echo "Az stepsize = " $STEP_AZ
echo "Alt stepsize = " $STEP_ALT
echo "Averages = " $AVERAGE

#exit 1 

az=$min_az
alt=$min_alt

# switch off tracking
echo -n $(indi_setprop "Pi Radiotelescope.TELESCOPE_TRACK_STATE.TRACK_OFF=On")

goto_pos $az $alt 100

while [ $(echo "scale=6; $az <= $max_az"|bc) -eq 1 ]
do
   # zuerst nach oben scannen
   alt=$min_alt
   while [ $(echo "scale=6; $alt <= $max_alt"|bc) -eq 1 ]
   do
      echo "az=" $az " alt=" $alt
      goto_pos $az $alt
      echo -n $(./rt_ads1115_measurement.sh 1 >> $5)

      alt=$(echo "scale=6; $alt+$STEP_ALT" | bc)
   done

   az=$(echo "scale=6; $az+$STEP_AZ" | bc)

   #dann nach unten scannen, um langen Rückfahrvorgang zu vermeiden
   alt=$max_alt
   while [ $(echo "scale=6; $alt >= $min_alt"|bc) -eq 1 ]
   do
      echo "az=" $az " alt=" $alt
      goto_pos $az $alt
      echo -n $(./rt_ads1115_measurement.sh 1 >> $5)

      alt=$(echo "scale=6; $alt-$STEP_ALT" | bc)
   done
   az=$(echo "scale=6; $az+$STEP_AZ" | bc)
done
