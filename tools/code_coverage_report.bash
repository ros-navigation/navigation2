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

for opt in "$@" ; do
  case "$opt" in
    clean)
      rm -rf install build log $LCOVDIR
      exit 0
      ;;
    codecovio)
      COVERAGE_REPORT=codecovio
      ;;
    genhtml)
      COVERAGE_REPORT=genhtml
      ;;
  esac
done

mkdir -p $LCOVDIR

# Re-enable branch coverage.
# This is disabled by default in the latest version of LCOV
lcov () {
  set -o xtrace
  command lcov $@ --rc lcov_branch_coverage=1
  set +o xtrace
}

# Generate initial zero-coverage data.
# This adds files that were otherwise not run to the report
lcov --capture --initial \
  --directory build \
  --output-file ${LCOVDIR}/initial_coverage.info

# Capture executed code data.
lcov --capture \
  --directory build \
  --output-file ${LCOVDIR}/test_coverage.info

# Combine the initial zero-coverage report with the executed lines report.
lcov \
  --add-tracefile ${LCOVDIR}/initial_coverage.info \
  --add-tracefile ${LCOVDIR}/test_coverage.info \
  --output-file ${LCOVDIR}/full_coverage.info

# Only include files that are within this workspace.
# (eg filter out stdio.h etc)
lcov \
  --extract ${LCOVDIR}/full_coverage.info \
    "${PWD}/*" \
  --output-file ${LCOVDIR}/workspace_coverage.info

# Remove files in the build subdirectory.
# Those are generated files (like messages, services, etc)
lcov \
  --remove ${LCOVDIR}/workspace_coverage.info \
    "${PWD}/build/*" \
  --output-file ${LCOVDIR}/project_coverage.info

if [ $COVERAGE_REPORT = codecovio ]; then
  bash <(curl -s https://codecov.io/bash) \
    -f ${LCOVDIR}/project_coverage.info \
    -R src/navigation2
elif [ $COVERAGE_REPORT = genhtml ]; then
  genhtml ${LCOVDIR}/project_coverage.info \
    --output-directory ${LCOVDIR}/html \
    --branch-coverage -p ${PWD}
fi
