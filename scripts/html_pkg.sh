#!/bin/bash

pip install beautifulsoup4
pip install selenium

wget https://github.com/mozilla/geckodriver/releases/download/v0.30.0/geckodriver-v0.30.0-linux64.tar.gz
tar -xvzf geckodriver*
sudo mv geckodriver /usr/local/bin/
cd /usr/local/bin/
sudo chmod +x geckodriver