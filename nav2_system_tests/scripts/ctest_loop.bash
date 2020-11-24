#!/bin/bash

#
# Simple bash script to loop over the navigation2 "bt_navigator" system test
#
# options:
# -c <#> - number of times to loop
# -o <file> - name of summary output file
# -l <file> - name to use for failing log files
# -d <dds> - name of DDS implementation to use (ex: rmw_fastrtps_cpp)

failcount=0
loopcount=1

while getopts c:o:l:d: option
  do
    case "$option"
    in
      c) loopcount=$OPTARG;;
      o) outfile=$OPTARG;;
      l) logfile=$OPTARG;;
      d) dds=$OPTARG;;
    esac
  done

echo "Total loop count = " $loopcount
export RMW_IMPLEMENTATION=$dds

for ((i=1; i<=$loopcount; i++))
  do
    echo "******************************"
    echo "Loop number: " $i
    echo "Running with DDS: " $dds
    echo "******************************"

    ctest -V -R test_bt_navigator$ -O $logfile
    result=$?
    echo "RESULT =" $result
    if [ "$result" != "0" ]
    then
      ((failcount+=1))
      echo "TEST $i FAILED" >> $outfile
      mv $logfile $logfile.$i.fail
    fi
    echo $i "TESTS COMPLETED"
    echo $failcount "TOTAL FAILURES"
  done
echo $failcount " FAILURES / " $loopcount
echo $failcount " FAILURES / " $loopcount >> $outfile

