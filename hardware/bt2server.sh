#!/bin/bash

tail -f screenlog.0 | grep --line-buffered -o '[0-9:]*\.[0-9]*' | while read ang
    do
        curl -X POST -d '{ "user": "Peter", "angle": "'${ang}'" }' http://localhost:6000/steps --header "Content-Type: application/json"
    done
