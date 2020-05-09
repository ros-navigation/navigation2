#!/bin/bash

if [ ! -d build ]; then
  echo "Please run this script from the root of your workspace."
  echo "Expected directory hierarchy is:"
  echo "example_ws"
  echo " - build"
  echo " - - package_a"
  echo " - - package_b"
  exit 1
fi

set -e

LCOVDIR=lcov
PWD=`pwd`

COVERAGE_REPORT_VIEW="genhtml"

for opt in "$@" ; do
  case "$opt" in
    clean)
      rm -rf install build log $LCOVDIR
      exit 0
      ;;
    codecovio)
      COVERAGE_REPORT_VIEW=codecovio
      ;;
    genhtml)
      COVERAGE_REPORT_VIEW=genhtml
      ;;
    ci)
      COVERAGE_REPORT_VIEW=ci
      ;;
  esac
done

set -o xtrace
mkdir -p $LCOVDIR

# Generate initial zero-coverage data.
# This adds files that were otherwise not run to the report
lcov --capture --initial \
  --directory build \
  --output-file ${LCOVDIR}/initial_coverage.info \
  --rc lcov_branch_coverage=0

# Capture executed code data.
lcov --capture \
  --directory build \
  --output-file ${LCOVDIR}/test_coverage.info \
  --rc lcov_branch_coverage=0

# Combine the initial zero-coverage report with the executed lines report.
lcov \
  --add-tracefile ${LCOVDIR}/initial_coverage.info \
  --add-tracefile ${LCOVDIR}/test_coverage.info \
  --output-file ${LCOVDIR}/full_coverage.info \
  --rc lcov_branch_coverage=0

# Only include files that are within this workspace.
# (eg filter out stdio.h etc)
lcov \
  --extract ${LCOVDIR}/full_coverage.info \
    "${PWD}/*" \
  --output-file ${LCOVDIR}/workspace_coverage.info \
  --rc lcov_branch_coverage=0

# Remove files in the build subdirectory.
# Those are generated files (like messages, services, etc)
# And system tests, which are themselves all test artifacts
lcov \
  --remove ${LCOVDIR}/workspace_coverage.info \
    "${PWD}/build/*" \
  --remove ${LCOVDIR}/workspace_coverage.info \
    "${PWD}/*/dwb_msgs/*" \
  --remove ${LCOVDIR}/workspace_coverage.info \
    "${PWD}/*/nav2_msgs/*" \
  --remove ${LCOVDIR}/workspace_coverage.info \
    "${PWD}/*/nav_2d_msgs/*" \
  --remove ${LCOVDIR}/workspace_coverage.info \
    "${PWD}/*/nav2_system_tests/*" \
  --output-file ${LCOVDIR}/project_coverage.info \
  --rc lcov_branch_coverage=0

if [ $COVERAGE_REPORT_VIEW = codecovio ]; then
  bash <(curl -s https://codecov.io/bash) \
    -f ${LCOVDIR}/project_coverage.info \
    -R src/navigation2
elif [ $COVERAGE_REPORT_VIEW = genhtml ]; then
  genhtml ${LCOVDIR}/project_coverage.info \
    --output-directory ${LCOVDIR}/html
fi
