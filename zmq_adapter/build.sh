#!/bin/bash
zip_file="zmq_adapter.zip"
temp_dir="./zmq_adapter"

rm -f "$zip_file"
mkdir -p "$temp_dir"

rsync -r --exclude='env' --exclude='__pycache__' --exclude='*.zip' ../zmq_adapter/ "$temp_dir"

chmod +x "$temp_dir/start.sh"

cd "$temp_dir" || exit 1
rm -r "$temp_dir"

cd ..
zip -r "$zip_file" "./$temp_dir"

rm -r "$temp_dir"
