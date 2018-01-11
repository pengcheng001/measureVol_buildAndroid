#!/bin/sh

cd ../../Wrapper4Neolix/
./build.sh
cd ../neolix_cubing_jni/pmd_sdk/
rm arm* -rf
cp ../../Wrapper4Neolix/obj/local/* . -nR
rm -r include
cp -r ../../Wrapper4Neolix/include/ .
