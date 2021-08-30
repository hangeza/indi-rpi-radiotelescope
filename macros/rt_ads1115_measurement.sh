#!/bin/bash
# show RT position and onboard adc values in a loop

# usage: ./rt_ads1115_measurement.sh <N> <DELAY>
# with the optional two arguments
# N: number of measurements in a loop, zero for endless loop
N=0
# DELAY: waiting time after each measurement in s
DELAY=0.0

if [ $# -eq 1 ]; then
   let N=$1
fi

if [ $# -eq 2 ]; then
   let N=$1
   let DELAY=$2
fi

cmd_indi="indi_getprop -t 5 -1"
prop_az="Pi Radiotelescope.HORIZONTAL_EOD_COORD.AZ"
prop_alt="Pi Radiotelescope.HORIZONTAL_EOD_COORD.ALT"
prop_ra="Pi Radiotelescope.EQUATORIAL_EOD_COORD.RA"
prop_dec="Pi Radiotelescope.EQUATORIAL_EOD_COORD.DEC"
prop_temp="Pi Radiotelescope.TEMPERATURE_MONITOR.TEMPERATURE1"
prop_adc1="Pi Radiotelescope.MEASUREMENTS.MEASUREMENT0"
prop_adc2="Pi Radiotelescope.MEASUREMENTS.MEASUREMENT1"
cmd_date="date '+%s.%N'"

# make number of measurements
count=0
while [[ $count -lt $N ]] || [[ $N == 0 ]]
do
  sleep $DELAY
  indi_props=$($cmd_indi "$prop_ra" "$prop_dec" "$prop_az" "$prop_alt" "$prop_temp" "$prop_adc1" "$prop_adc2" 2>/dev/null)
  for i in $indi_props; do
    if echo $i | grep -qe 'EQUATORIAL_EOD_COORD.RA'
    then
      ra=$(echo $i | awk -F"=" '{ printf "%1.5f", strtonum($(NF)) }')
    fi
    if echo $i | grep -qe 'EQUATORIAL_EOD_COORD.DEC'
    then
      dec=$(echo $i | awk -F"=" '{ printf "%1.4f", strtonum($(NF)) }')
    fi
    if echo $i | grep -qe 'HORIZONTAL_EOD_COORD.AZ'
    then
      az=$(echo $i | awk -F"=" '{ printf "%1.4f", strtonum($(NF)) }')
    fi
    if echo $i | grep -qe 'HORIZONTAL_EOD_COORD.ALT'
    then
      alt=$(echo $i | awk -F"=" '{ printf "%1.4f", strtonum($(NF)) }')
    fi
    if echo $i | grep -qe 'TEMPERATURE1'
    then
      temp=$(echo $i | awk -F"=" '{ printf "%1.1f", strtonum($(NF)) }')
    fi
        if echo $i | grep -qe 'MEASUREMENTS.MEASUREMENT0'
    then
      adc1=$(echo $i | awk -F"=" '{ printf "%1.4f", strtonum($(NF)) }')
    fi
    if echo $i | grep -qe 'MEASUREMENTS.MEASUREMENT1'
    then
      adc2=$(echo $i | awk -F"=" '{ printf "%1.4f", strtonum($(NF)) }')
    fi
  done
  echo $(eval $cmd_date) $az $alt $ra $dec $adc1 $adc2 $temp
  let count=$count+1
done
