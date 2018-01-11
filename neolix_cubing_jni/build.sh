#!/bin/bash

cd neolixAlgorithm/
./update_sdk.sh
cd ../pmd_sdk/
./ipdate_sdk.sh
cd ../
./clean.sh
ndk-build NDK_PROJECT_PATH=. NDK_APPLICATION_MK=Application.mk
