ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE} AS build-stage

ARG PJ_TAG=3.9.2

###############################################################################
# Install dependencies.
###############################################################################
# General dependencies.
RUN apt update && apt install -y build-essential cmake git 
# PlotJuggler dependencies.
RUN apt update && apt -y install qtbase5-dev libqt5svg5-dev libqt5websockets5-dev \
    libqt5opengl5-dev libqt5x11extras5-dev libprotoc-dev libzmq3-dev \
    liblz4-dev libzstd-dev

###############################################################################
# Compile PlotJuggler
###############################################################################
WORKDIR /plotjuggler_ws/src
RUN git clone --depth 1 --branch ${PJ_TAG} https://github.com/facontidavide/PlotJuggler.git PlotJuggler
# Build PlotJuggler. This will take a long time.
WORKDIR /plotjuggler_ws
RUN cmake -S src/PlotJuggler -B build/PlotJuggler -DCMAKE_INSTALL_PREFIX=install \
    && cmake --build build/PlotJuggler --config RelWithDebInfo --target install

###############################################################################
# Compile the plugin
###############################################################################
ARG ADD_UNITS=OFF

COPY --link . /apbin_plugin
WORKDIR /apbin_plugin/build
# Ensure a fresh build folder
RUN rm -R * \
    && cmake -Dplotjuggler_DIR="/plotjuggler_ws/install/lib/cmake/plotjuggler" -DADD_UNITS=${ADD_UNITS} .. \
    && make \
    && make install \
    && mkdir /artifacts \
    && cp libDataAPBin.so /artifacts

###############################################################################
# Export the plugin
###############################################################################
FROM scratch AS export-stage
# Move the plugin to a fresh filesystem that can be exported easily.
COPY --from=build-stage /artifacts /
