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

COVERAGE_REPORT=genhtml

for opt in "$@" ; do
  case "$opt" in
    clean)
      rm -rf install build log $LCOVDIR
      exit 0
      ;;
    codecovio)
      COVERAGE_REPORT=codecovio
      ;;
  esac
done

mkdir -p $LCOVDIR

# Generate initial zero-coverage data. This adds files that were otherwise not run to the report
lcov -c  --initial --rc lcov_branch_coverage=1 --directory build --output-file ${LCOVDIR}/initialcoverage.info

# Capture executed code data.
lcov -c --rc lcov_branch_coverage=1 --directory build --output-file ${LCOVDIR}/testcoverage.info

# Combine the initial zero-coverage report with the executed lines report
lcov -a ${LCOVDIR}/initialcoverage.info -a ${LCOVDIR}/testcoverage.info --rc lcov_branch_coverage=1 --o ${LCOVDIR}/fullcoverage.info

# Only include files that are within this workspace (eg filter out stdio.h etc)
lcov -e ${LCOVDIR}/fullcoverage.info "${PWD}/*" --rc lcov_branch_coverage=1 --output-file ${LCOVDIR}/workspacecoverage.info

# Remove files in the build subdirectory because they are generated files (like messages, services, etc)
lcov -r ${LCOVDIR}/workspacecoverage.info "${PWD}/build/*" --rc lcov_branch_coverage=1 --output-file ${LCOVDIR}/projectcoverage.info

if [ $COVERAGE_REPORT = codecovio ]; then
  bash <(curl -s https://codecov.io/bash) -f ${LCOVDIR}/projectcoverage.info
else
  genhtml ${LCOVDIR}/projectcoverage.info --output-directory ${LCOVDIR}/html --branch-coverage -p ${PWD}
fi
