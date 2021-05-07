# POST Box ATSAMD21E18A firmware

## Recommended

To build without messing with any of your Arduino libraries, and store
the built code in the binaries/ folder:

  python3 docker_build.py

To upload built code from the binaries/ folder:

  POSTBOX_BUILD=none python3 build_and_upload.py

(Yes, this is a bit awkward.  I'm going to split that into two scripts
sometime!)

## For development only

Local dev build, if you don't mind having a different (possibly old,
depending when you read this) version of adafruit:arduino installed:

  POSTBOX_PORT=none POSTBOX_BUILD=/tmp/pb python3 build_and_upload.py

Local dev build and upload to connected POST box -- or possibly a
connected Circuit Playground; you probably want to unplug any other
Arduino devices or Arcflash boards before running this:

  POSTBOX_BUILD=/tmp/pb python3 build_and_upload.py
