#!/usr/bin/env python3
from pathlib import Path
import argparse
import shutil
import re

# leveraging the difficult work done by rtl_433_ESP
# this script automates copy from a populated rtl_433 directory... it doesn't do any error checking
# always review the results
copy_exact="""include/c_util.h include/abuf.h include/bitbuffer.h include/compat_time.h 
include/decoder.h include/decoder_util.h include/fatal.h include/logger.h 
include/optparse.h include/output_log.h include/pulse_detect.h include/pulse_slicer.h 
include/r_api.h include/r_device.h include/r_util.h include/rfraw.h include/util.h
include/data.h include/bit_util.h
src/abuf.c src/bitbuffer.c src/compat_time.c src/data.c src/decoder_util.c
src/logger.c src/output_log.c src/pulse_data.c src/r_util.c src/util.c src/rfraw.c
src/devices/*.c
""".split()

# These probably can be fixed to work if need be with a little work...
exclude_list=set("""blueline deltadore_x3d rosstech_dcu706 secplus_v2""".split())

# forked: manually review for updates
#src/r_api.c
#include/pulse_data.h
#include/r_private.h
#include/rtl_433.h

# temporary fork (https://github.com/juanboro/esphome-rtl_433-decoder/issues/3)
# include/list.h
# src/list.c

# todo - snapshot rtl_433 git repo version

def do_copy_exact(rtl433dir,outdir):
    for filepattern in copy_exact:
        for srcfilename in rtl433dir.glob(filepattern):
            outfilename=outdir / srcfilename.relative_to(rtl433dir).as_posix().replace("src/","src/rtl_433/")
            shutil.copy(srcfilename,outfilename)

def gen_devices(outdir):
    devre=re.compile(r"r_device\s+(?:const\s+)?([^\s=]+)\s*=")
    for outdevicefile in outdir.glob("src/rtl_433/devices/*.c"):
        for line in outdevicefile.open("rt"):
            devmatch=devre.match(line)
            if devmatch is not None: 
                devname=devmatch.groups()[0]
                if devname not in exclude_list: yield devname

def update_rtl_433_devices(outdir):
    # create a new rtl_433_devices.h
    outfile=outdir / "include/rtl_433_devices.h"
    with outfile.open("wt") as outfp:
        outfp.write("""/** @file
    This is a generated file from tools/update_rtl433.py
*/

#ifndef INCLUDE_RTL_433_DEVICES_H_
#define INCLUDE_RTL_433_DEVICES_H_

#include "r_device.h"

#ifdef MY_RTL433_DEVICES
#define DEVICES MY_RTL433_DEVICES
#else
#define DEVICES                      \\\n""")
        for outdevice in sorted(gen_devices(outdir),key=str.lower):
            outfp.write("    %-33s\\\n" % f"DECL({outdevice})")
        outfp.write("""
#endif
#define DECL(name) extern r_device name;
DEVICES
#undef DECL

#endif /* INCLUDE_RTL_433_DEVICES_H_ */
""")
                    

def main():
    rtl433dir=Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser()
    parser.add_argument("rtl433dir",help="rtl_433 directory")
    parser.add_argument("outdir", nargs='?',default=rtl433dir.as_posix(),help='output directory')
    args = parser.parse_args()
    rtl433dir=Path(args.rtl433dir)
    outdir=Path(args.outdir)

    # pulse_slicer.c: fix bitbuffer init to be portable and global because it's too big to allocate on the task stack
    srcfile=rtl433dir / "src/pulse_slicer.c"
    outfile=outdir / "src/rtl_433/pulse_slicer.c"
    outfile.write_text(srcfile.read_text()
                       .replace("bitbuffer_t bits = {0};","bitbuffer_clear(&bits);")
                       .replace('#include <limits.h>',"#include <limits.h>\nbitbuffer_t bits;"))

    do_copy_exact(rtl433dir,outdir)
    update_rtl_433_devices(outdir)

if __name__ == "__main__":
    main()

