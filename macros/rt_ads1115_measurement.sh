#!/bin/bash
# show RT position and onboard adc values in a loop

# settling time after switching in s
DELAY1=0.5
N=0

if [ $# -eq 1 ]; then
   let N=$1
fi

cmd_indi="indi_getprop -t 1 -1"
prop_az="Pi Radiotelescope.HORIZONTAL_EOD_COORD.AZ"
prop_alt="Pi Radiotelescope.HORIZONTAL_EOD_COORD.ALT"
prop_ra="Pi Radiotelescope.EQUATORIAL_EOD_COORD.RA"
prop_dec="Pi Radiotelescope.EQUATORIAL_EOD_COORD.DEC"
prop_temp="Pi Radiotelescope.TEMPERATURE_MONITOR.TEMPERATURE1"
prop_adc1="Pi Radiotelescope.VOLTAGE_MONITOR.VOLTAGE4"
prop_adc2="Pi Radiotelescope.VOLTAGE_MONITOR.VOLTAGE5"
cmd_date="date '+%s.%N'"

# make number of measurements
count=0
while [[ $count -lt $N ]] || [[ $N == 0 ]]
do
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
        if echo $i | grep -qe 'VOLTAGE_MONITOR.VOLTAGE4'
    then
      adc1=$(echo $i | awk -F"=" '{ printf "%1.4f", strtonum($(NF)) }')
    fi
    if echo $i | grep -qe 'VOLTAGE_MONITOR.VOLTAGE5'
    then
      adc2=$(echo $i | awk -F"=" '{ printf "%1.4f", strtonum($(NF)) }')
    fi
  done
  echo $(eval $cmd_date) $az $alt $ra $dec $adc1 $adc2 $temp
 let count=$count+1
done
