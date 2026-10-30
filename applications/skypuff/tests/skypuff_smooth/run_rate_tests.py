#!/usr/bin/env python3
"""Check acceleration holds in the current rate-based production controller."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
from plot_rate_transitions import ROOT, prepare_production_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'gcc'))
    parser.add_argument('--output', type=Path, default=ROOT/'build/skypuff_rate_pause_tests')
    args = parser.parse_args()
    output = args.output.resolve()
    prepare_production_code(output)
    for debug in (False, True):
        executable = output/('rate_pause_debug' if debug else 'rate_pause')
        flags = ['-DDEBUG_SMOOTH_MOTOR'] if debug else []
        subprocess.run(shlex.split(args.cc)+flags+['-std=gnu99', '-O2', '-Wall', '-Wextra', '-Werror',
            '-Wno-old-style-declaration', '-fsanitize=undefined', '-fno-sanitize-recover=all',
            '-I', str(output), str(Path(__file__).with_name('rate_pause_harness.c')),
            '-lm', '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
