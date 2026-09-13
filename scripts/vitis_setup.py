#!/usr/bin/env python3
"""vitis_setup.py — create the bare-metal platform and application for L4.

Vitis 2025.2 unified flow. Run it with the Vitis Python interpreter, not the
system one:

    cd <project root>
    make board_vectors                      # the frame and golden arrays
    /c/AMDDesignTools/2025.2/Vitis/bin/vitis -s scripts/vitis_setup.py \\
        --xsa build/zybo/dcc_enc.xsa --workspace build/vitis

Then either build from the command line (this script does it) or open the
workspace in the IDE:

    /c/AMDDesignTools/2025.2/Vitis/bin/vitis -w build/vitis

What it builds
--------------
A standalone domain on ps7_cortexa9_0, and one application whose sources are
flattened into a single directory: the board application, the codec-agnostic
driver, the H.264 host module, and -- the point of the whole exercise --
src/nal.c and src/bitstream.c verbatim from the reference encoder. The board
emits its parameter sets and slice header with the same code that produced the
golden stream on the workstation, so the container cannot disagree with itself.

Flattening the sources means no include directories are needed and the
#include "nal.h" style in the host files resolves as-is.
"""

import argparse
import os
import shutil
import sys

import vitis

# (source directory, files) — all land flat in the component's src/.
SOURCES = [
    ("host", ["board_main.c", "codec_kernel.c", "codec_kernel.h",
              "h264_host.c", "h264_host.h", "platform_standalone.c"]),
    ("src",  ["nal.c", "nal.h", "bitstream.c", "bitstream.h", "types.h"]),
    ("build/board", ["frame_data.c", "golden_data.c"]),
]

PLATFORM = "dcc_plat"
APP = "dcc_l4"
CPU = "ps7_cortexa9_0"
DOMAIN = "standalone_ps7_cortexa9_0"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xsa", default="build/zybo/dcc_enc.xsa")
    ap.add_argument("--workspace", default="build/vitis")
    ap.add_argument("--root", default=".", help="project root")
    ap.add_argument("--no-build", action="store_true")
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    xsa = os.path.abspath(args.xsa)
    ws = os.path.abspath(args.workspace)

    if not os.path.isfile(xsa):
        sys.exit("no XSA at %s -- run `make zybo` first" % xsa)

    # Check every source exists before touching the workspace, so a missing
    # generated file fails in a sentence rather than halfway through a build.
    missing = [os.path.join(d, f)
               for d, files in SOURCES for f in files
               if not os.path.isfile(os.path.join(root, d, f))]
    if missing:
        sys.exit("missing sources:\n  " + "\n  ".join(missing) +
                 "\n(build/board/* come from `make board_vectors`)")

    if os.path.isdir(ws):
        shutil.rmtree(ws)
    os.makedirs(ws, exist_ok=True)

    client = vitis.create_client()
    client.set_workspace(ws)

    print("creating platform from %s" % xsa)
    platform = client.create_platform_component(
        name=PLATFORM, hw_design=xsa, os="standalone",
        cpu=CPU, domain_name=DOMAIN)
    platform.build()

    xpfm = client.find_platform_in_repos(PLATFORM)
    print("platform: %s" % xpfm)

    client.create_app_component(name=APP, platform=xpfm, domain=DOMAIN,
                                template="empty_application")
    comp = client.get_component(name=APP)

    for d, files in SOURCES:
        comp.import_files(from_loc=os.path.join(root, d), files=files)
        print("imported %d files from %s" % (len(files), d))

    # -O2 matters here: the staging memcpy and the byte-wise payload compare
    # are the only things the CPU does per frame, and at -O0 they dominate the
    # timing numbers the run reports.
    comp.set_app_config(key="USER_COMPILE_OPTIMIZATION_LEVEL", values="-O2")

    if not args.no_build:
        print("building...")
        comp.build()
        print("built. ELF under %s" % os.path.join(ws, APP, "build"))

    print("""
next:
  1. connect the Zybo, set JP5 to JTAG, power on
  2. open the workspace:  vitis -w %s
  3. Run > Run As > Launch on Hardware, with the bitstream programmed
     (the platform carries it; enable "Program FPGA" in the run config)
  4. watch UART at 115200 8N1
""" % ws)
    vitis.dispose()


if __name__ == "__main__":
    main()
