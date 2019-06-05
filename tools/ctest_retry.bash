#!/bin/bash

usage() {
  echo "ctest_retry.bash [options]"
  echo "Reruns ctest until the test passes or it has run 3 times (by default)."
  echo "  -r <number> Retry the test up to <number> times [Default: 3]"
  echo "  -d <path> Execute ctest from the given path [Default: Current working directory]"
  echo "  -t <testname> Execute only <testname> [Default: execute all tests available in <path> ]"
  echo "  -h displays this usage summary"
  exit 0
}

RETRIES=3
TESTDIR=.
SPECIFIC_TEST=""

# Check Options
while getopts ":hr:d:t:" opt; do
  case "$opt" in
    h)
      usage
      ;;
    r)
      RETRIES=$OPTARG
      ;;
    d)
      TESTDIR=$OPTARG
      ;;
    t)
      SPECIFIC_TEST="-R $OPTARG"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

total=$RETRIES
cd $TESTDIR
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1

echo "Retrying Ctest up to " $total " times."
for ((i=1;i<=total;i++))
  do
    ctest -V $SPECIFIC_TEST
    result=$?
    if [ "$result" == "0" ]  # if ctest succeeded, then exit the retry loop
    then
      echo "Test succeeded on try " $i
      exit 0
    fi
  done
echo "Test failed " $total " times."
exit $result
