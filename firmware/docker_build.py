import subprocess

import config

def cmd(s):
    return subprocess.check_call(s, shell=True)

cmd("docker build --build-arg UPSTREAM_CORE=%s --tag=myelin_postbox_arduino ." % config.UPSTREAM_CORE)
cmd("docker run --rm -it --mount type=bind,source=$(pwd)/..,target=/src myelin_postbox_arduino:latest 'cd firmware && POSTBOX_PORT=none POSTBOX_BUILD=/tmp/pb python3 build_and_upload.py'")
