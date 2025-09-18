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

git submodule init && git submodule update

if [[ "$arch" == "armv6l" || "$arch" == "armv7l" ]]
then
  echo "system = Arm-linux-gnueabihf" > VZense_python_wrapper/config.txt
  echo "url = https://gitee.com" >> VZense_python_wrapper/config.txt
else
  echo "system = Ubuntu18.04" > VZense_python_wrapper/config.txt
  echo "url = https://gitee.com" >> VZense_python_wrapper/config.txt
fi
cd VZense_python_wrapper && python install.py && rm -rf tmp; cd ../

echo "Done!"
