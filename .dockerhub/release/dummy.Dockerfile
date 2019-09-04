# This is a dummy Dockerfile for repository links on Docker Hub.
# Build rules on Docker Hub can trigger whenever the base image updates.
# Base images are specified in the FROM: directive in the tracked Dockerfile.
# However, build args are used by the build hooks to adjust the base image
# so a single Dockerfile can be reused for multiple CI jobs, reducing maintenance.
# To re-enable repository linking when using build args in the FROM: directive,
# this dummy Dockerfile explicitly conveys the base image/repo to link against
# while build rules that target this still use the same hook in this directory.
# Note: This only works for non-official images.

FROM osrf/ros2:nightly-rmw-nonfree
RUN echo "This is a dummy Dockerfile."