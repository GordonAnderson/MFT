"""
PlatformIO post-build step: copies the built firmware.hex into firmware/,
named MFT_V<version>.hex, where <version> is parsed straight out of the
Version[] string in src/MFT.cpp, so the published file always matches what the
firmware reports via GVER. Invoked from platformio.ini (extra_scripts).
"""
Import("env")

import os
import re

PROJECT_DIR = env["PROJECT_DIR"]
SRC_FILE = os.path.join(PROJECT_DIR, "src", "MFT.cpp")
FIRMWARE_DIR = os.path.join(PROJECT_DIR, "firmware")

VERSION_RE = re.compile(r'Version\[\]\s*PROGMEM\s*=\s*"MFT version ([^",]+)')


def publish_firmware(source, target, env):
    with open(SRC_FILE, "r") as f:
        m = VERSION_RE.search(f.read())
    if not m:
        print("firmware publish: could not find Version[] in src/MFT.cpp - skipped")
        return
    version = m.group(1).strip()          # e.g. "1.9"

    with open(str(target[0]), "rb") as f:
        data = f.read()

    os.makedirs(FIRMWARE_DIR, exist_ok=True)
    filename = "MFT_V{}.hex".format(version)
    dest = os.path.join(FIRMWARE_DIR, filename)

    if os.path.isfile(dest):
        with open(dest, "rb") as f:
            if f.read() != data:
                print("firmware publish: WARNING - overwriting {} with DIFFERENT content "
                      "for the same version. Did you forget to bump Version[] in "
                      "src/MFT.cpp?".format(filename))

    with open(dest, "wb") as f:
        f.write(data)
    print("firmware publish: {} ({} bytes)".format(dest, len(data)))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", publish_firmware)
