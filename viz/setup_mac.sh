#!/bin/bash
# 
# Usage:
#    bash setup_mac.sh
#
#
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
sudo easy_install pip
sudo pip install networkx 
sudo pip install requests
sudo pip install matplotlib 
sudo pip install configobj 
sudo pip install mpltools