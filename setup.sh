#!/bin/bash
set -e

if [["$(uname -m)" == "armv6l"]]
then
  sudo apt install python3-opencv
else
  echo "If opencv takes ages to install, you might want to look for binaries for it."
fi

python -m pip install -r requirements.txt

git submodule init && git submodule update

cd VZense_python_wrapper && python install.py; cd ../

echo "Done!"
