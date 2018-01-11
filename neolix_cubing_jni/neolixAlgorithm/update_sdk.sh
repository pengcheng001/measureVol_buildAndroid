#!/bin/sh
rm lib/* include/* -rf
cd ../../algrothmic/
./build.sh
cd ../neolix_cubing_jni/neolixAlgorithm/
cp ../../algrothmic/obj/local/armeabi-v7a ./lib/armeabi-v7a/ -nR
cp ../../algrothmic/neolixai/neolixMV.h ./include/
