#!/bin/bash

mkdir -p /app
cd /app

stat main.py > /dev/null 2>&1
if [ $? -ne 0 ]; then
    set -e
    echo "Downloading source..."
    curl --fail -H "Authorization: ${AUTH_HEADER}" ${PROJECT_URL}/main.py -o main.py > /dev/null 2>&1
    curl --fail -H "Authorization: ${AUTH_HEADER}" ${PROJECT_URL}/props.json -o props.json > /dev/null 2>&1
else
    echo "Using mounted source..."
fi

set +e
stat props.json > /dev/null 2>&1
if [ $? -eq 0 ]; then
    requirements=$(/px -ijson "loads(''.join(x)).get('pip_requirements') or ''" props.json)
    if [ $? -ne 0 ]; then
        requirements=""
    fi
fi
stat props.yaml > /dev/null 2>&1
if [ $? -eq 0 ]; then
    requirements=$(/px -iyaml "safe_load('\n'.join(x)).get('pip_requirements') or ''" props.yaml)
    if [ $? -ne 0 ]; then
        requirements=""
    fi
fi
if [ -z "$requirements" ]; then
    echo "No pip requirements found"
else
    echo "Requirements found: $requirements"
    set -e
    pip install $requirements
fi

set -e
python3 -u main.py
