#!/bin/bash

BOT_DIR="$(dirname $(dirname "$0"))/$1"

cd $BOT_DIR

cp template.html index.html

python3 -m http.server
