#!/bin/bash

git submodule update --init --recursive

cd external/zcm_types && ./build.sh
