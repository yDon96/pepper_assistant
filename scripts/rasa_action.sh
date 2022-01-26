#!/bin/bash

BOT_DIR="$(dirname $(dirname "$0"))/$1"

cd $BOT_DIR

rasa run actions