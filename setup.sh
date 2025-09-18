#!/bin/bash
set -e

arch=$(uname -m)
if [[ "$arch" == "armv6l" || "$arch" == "armv7l" ]]
then
  sudo apt install python3-opencv
else
  echo "If opencv takes ages to install, you might want to look for binaries for it."
fi

python -m pip install -r requirements.txt

git submodule update --init

if [[ "$arch" == "armv6l" || "$arch" == "armv7l" ]]
then
  subfolder='Arm-linux-gnueabihf'
else
  subfolder='Ubuntu18.04'
fi
echo "system = ${subfolder}" > VZense_python_wrapper/config.txt
echo "url = https://github.com" >> VZense_python_wrapper/config.txt

cd VZense_python_wrapper && rm -rf tmp && python install.py; cd ../
cd VZense_python_wrapper/tmp/Vzense_SDK_linux/${subfolder} && sudo ./install.sh; cd ../../../../

echo "Done!"
