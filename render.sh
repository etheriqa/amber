#!/bin/sh

set -ex

make -j
./amber
convert output.ppm output.png
which open > /dev/null && open output.png
