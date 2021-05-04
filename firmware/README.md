# POST Box ATSAMD21E18A firmware

To build without messing with any of your Arduino libraries:

  python3 docker_build.py

Local dev build, if you don't mind having a different (possibly old,
depending when you read this) version of adafruit:arduino installed:

  POSTBOX_PORT=none POSTBOX_BUILD=/tmp/pb python3 build_and_upload.sh

Local dev build and upload to connected POST box -- or possibly a
connected Circuit Playground; you probably want to unplug any other
Arduino devices or Arcflash boards before running this:

  POSTBOX_BUILD=/tmp/pb python3 build_and_upload.sh