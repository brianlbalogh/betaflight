#!/bin/bash

targets=("PUBLISHMETA=True" \
<<<<<<< HEAD
     "TARGET=REVO" \
     "TARGET=REVO_OPBL" \
     "TARGET=SPARKY2" \
     "TARGET=SPARKY2_OPBL" \
     "TARGET=REVONANO" \
     "TARGET=REVONANO_OPBL" \
     "TARGET=ALIENFLIGHTF4" \
     "TARGET=BLUEJAYF4" \
     "TARGET=VRCORE" \
     "TARGET=AQ32_V2" \
     "TARGET=QUANTON" \
     "TARGET=QUANTON_OPBL")
#    "TARGET=CC3D" \
#    "TARGET=CC3D_OPBL" \
#    "TARGET=COLIBRI_RACE" \
#    "TARGET=LUX_RACE" \
#    "TARGET=SPRACINGF3" \
#    "TARGET=SPRACINGF3EVO" \
#    "TARGET=SPRACINGF3MINI" \
#    "TARGET=NAZE" \
#    "TARGET=AFROMINI" \
#    "TARGET=RMDO" \
#    "TARGET=SPARKY" \
#    "TARGET=MOTOLAB" \
#    "TARGET=IRCFUSIONF3" \
#    "TARGET=ALIENFLIGHTF1" \
#    "TARGET=ALIENFLIGHTF3" \
#    "TARGET=DOGE")

#fake a travis build environment
export TRAVIS_BUILD_NUMBER=$(date +%s)
export BUILDNAME=${BUILDNAME:=fake_travis}
export TRAVIS_REPO_SLUG=${TRAVIS_REPO_SLUG:=$USER/simulated}

for target in "${targets[@]}"
do
	unset RUNTESTS PUBLISHMETA TARGET
	eval "export $target"
	make -f Makefile clean
	./.travis.sh
done
