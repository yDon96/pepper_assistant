#!/bin/bash

BOT_DIR="$(dirname $(dirname "$0"))/$1"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
