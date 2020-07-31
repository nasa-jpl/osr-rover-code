#!/bin/bash

# exit on error
set -e

# directory of this script
DIR=$(realpath $(dirname $0))

pushd $DIR/..
pandoc README.md -t html5 -o README.pdf
popd