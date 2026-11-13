ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE} AS build-stage
# Build tools and Qt Core. Conan lives in a virtual environment so this also
# works on distributions whose system Python is externally managed.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates python3 python3-venv qt6-base-dev \
    && python3 -m venv /opt/conan \
    && /opt/conan/bin/python -m pip install 'conan>=2,<3'
ENV PATH="/opt/conan/bin:${PATH}"
WORKDIR /apbin_plugin
COPY . .
ARG ADD_UNITS=OFF
ARG ADD_RCOU_FUNCTION_LABELS=OFF
# Compile and test the plugin against the SDK, then collect the export artifacts.
RUN conan profile detect \
    && conan remote add plotjuggler https://plotjuggler.jfrog.io/artifactory/api/conan/plotjuggler-conan \
    && conan install . -of build --build=missing -s build_type=Release -s compiler.cppstd=20 \
    && cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake \
       -DCMAKE_BUILD_TYPE=Release -DADD_UNITS=${ADD_UNITS} -DADD_RCOU_FUNCTION_LABELS=${ADD_RCOU_FUNCTION_LABELS} \
    && cmake --build build --parallel \
    && ctest --test-dir build --output-on-failure \
    && mkdir /artifacts \
    && cp build/libDataAPBin.so build/DataAPBin.pjmanifest.json /artifacts/
FROM scratch AS export-stage
# Move the plugin to a fresh filesystem that can be exported easily.
COPY --from=build-stage /artifacts/ /
