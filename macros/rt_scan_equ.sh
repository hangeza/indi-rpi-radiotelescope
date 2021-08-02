#!/bin/bash
# scan equatorial window in a grid and take measurement at each point
# 02.08.2021 HGZ

# nr of values to acquire for one point
AVERAGE=50
# Timeout for waiting until RT reached set position in seconds
TIMEOUT=300
#step size in degrees
STEP_RA_DEFAULT=1.0
STEP_DEC_DEFAULT=1.0

cmd_stop='indi_setprop "Pi Radiotelescope.TELESCOPE_ABORT_MOTION.ABORT=On"'

prop_ra='"Pi Radiotelescope.EQUATORIAL_EOD_COORD.RA"'
prop_dec='"Pi Radiotelescope.EQUATORIAL_EOD_COORD.DEC"'

# function goto_pos()
# go to given coordinates and wait until reached there
# goto_pos ra dec [timeout]
# ra,dec...coordinate to go
# timeout...optional timeout after which the function returns 
goto_pos()
{
        _ra=$1
        _dec=$2

        if [[ $(echo "scale=6; $1 < 0.0"|bc) -eq 1 ]]; then
                _ra=$(echo "scale=6; $1+24.0" | bc)
        elif [[ $(echo "scale=6; $1 > 24.0"|bc) -eq 1 ]]; then
                _ra=$(echo "scale=6; $1-24.0" | bc)
        fi

        tout="$TIMEOUT"
        if [ "$3" ] # Is parameter #3 !zero length?
   then
                tout="$3"
   fi

        # goto position
        echo -n $(indi_setprop "Pi Radiotelescope.EQUATORIAL_EOD_COORD.RA;DEC=$_ra;$_dec")
   sleep 1
   indi_eval -w '"Pi Radiotelescope.SCOPE_STATUS.SCOPE_IDLE"==1'
}



if [ $# -lt 5 ]; then
  echo "Scan equatorial window in a grid and take measurement at each point"
  echo "need at least 5 arguments (equatorial coordinate boundaries and filename) for scan"
  echo "usage: $0 <min_ra> <max_ra> <min_dec> <max_dec> <file> [<step_ra> <step_dec> <avg>]"
  echo " parameters:"
  echo "  min_ra, min_dec - coordinate of lower left corner"
  echo "  max_ra, max_dec - coordinate of upper right corner"
  echo "  file - output file name"
  echo "  step_ra, step_dec - (optional) step size of coordinates in degree (default 1)"
  echo "  avg - (optional) take average of avg samples at each point (default 100)"
  exit 1
fi

if [[ $(echo "scale=6; $3 > $4"|bc) -eq 1 ]]; then
  echo "error: min dec > max dec"
  echo "exiting"
  exit 1
fi

if [[ $(echo "scale=6; $4 > 90.0"|bc) -eq 1 ]]; then
  echo "error: max dec out of range"
  echo "exiting"
  exit 1
fi

if [[ $(echo "scale=6; $1 > $2"|bc) -eq 1 ]]; then
  echo "error: min ra > max ra"
  echo "exiting"
  exit 1
fi

STEP_RA=$STEP_RA_DEFAULT
STEP_DEC=$STEP_DEC_DEFAULT

if [ $# -gt 5 ]; then
        STEP_RA=$6
fi

if [ $# -gt 6 ]; then
        STEP_DEC=$7
fi

if [ $# -gt 7 ]; then
        AVERAGE=$8
fi

min_az=$1
max_az=$2
min_alt=$3
max_alt=$4

echo $(eval $cmd_stop)

echo "southern boundary = " $min_dec
echo "northern boundary = " $max_dec
echo "eastern boundary = " $min_ra
echo "western boundary = " $max_ra
echo "Ra stepsize = " $STEP_RA
echo "Dec stepsize = " $STEP_DEC
echo "Averages = " $AVERAGE

#exit 1 

ra=$min_ra
dec=$min_dec

# switch off tracking
echo -n $(indi_setprop "Pi Radiotelescope.TELESCOPE_TRACK_STATE.TRACK_OFF=On")

goto_pos $ra $dec 100

while [ $(echo "scale=6; $ra <= $max_ra"|bc) -eq 1 ]
do
   # zuerst nach oben scannen
   alt=$min_dec
   while [ $(echo "scale=6; $dec <= $max_dec"|bc) -eq 1 ]
   do
      echo "ra=" $ra " dec=" $dec
      goto_pos $ra $dec
      echo -n $(./rt_ads1115_measurement.sh 1 >> $5)

      dec=$(echo "scale=6; $alt+$STEP_DEC" | bc)
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
