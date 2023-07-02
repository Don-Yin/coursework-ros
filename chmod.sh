#!/bin/bash

dir_path="src/robot/src"

for file in "$dir_path"/*.py
do
    chmod +x "$file"
done
