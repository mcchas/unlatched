"""Print the ESP Web Tools manifest for the web flasher (flasher/index.html).

    python3 scripts/flasher_manifest.py v0.1.0 > _site/manifest.json

The CI workflow runs this when it deploys the flasher to GitHub Pages. To try
the flasher locally (Web Serial works on localhost without HTTPS):

    pio run -e XiaoC6RF231
    mkdir -p _site && cp -r flasher/. _site/
    cp .pio/build/XiaoC6RF231/firmware.factory.bin _site/
    python3 scripts/flasher_manifest.py dev > _site/manifest.json
    python3 -m http.server -d _site 8000
Format: https://esphome.github.io/esp-web-tools/#manifest

new_install_prompt_erase: ESP Web Tools treats any device that does not answer
Improv as a new install, and erases a new install by default. This firmware
does not speak Improv, so without this flag every flash -- updates included --
would wipe NVS: the rig's node address, its stored keys and its Wi-Fi. With it
the user gets an "Erase device" checkbox, unticked by default.

new_install_improv_wait_time: 0 skips the wait for an Improv reply after
flashing, for the same reason.
"""

import json
import sys


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit("usage: flasher_manifest.py <version>")
    manifest = {
        "name": "Unlatched",
        "version": sys.argv[1],
        "new_install_prompt_erase": True,
        "new_install_improv_wait_time": 0,
        "builds": [
            {
                "chipFamily": "ESP32-C6",
                "parts": [{"path": "firmware.factory.bin", "offset": 0}],
            }
        ],
    }
    json.dump(manifest, sys.stdout, indent=2)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
