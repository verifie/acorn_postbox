# The original plan was to use Alpine, but I ran into issues with the arduino-cli installer.
FROM ubuntu:20.04

# Install deps: arduino-cli, python3, PySerial
RUN apt-get update \
    && apt-get install -y curl python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
RUN python3 -m pip install --user PySerial

# Install the correct version of the Adafruit core now, to speed up repeated runs of build_and_upload.py
ARG UPSTREAM_CORE
RUN arduino-cli --additional-urls=https://adafruit.github.io/arduino-board-index/package_adafruit_index.json core install ${UPSTREAM_CORE}

# Setup execution environment
WORKDIR /src
ENTRYPOINT ["/bin/bash", "-c"]
