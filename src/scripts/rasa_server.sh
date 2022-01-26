#!/bin/bash

BOT_DIR="/home/alex/Desktop/midTerm_project/midterm_RobShop"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
