#!/bin/bash
export PYTHONUNBUFFERED=true
pip3 install --upgrade protobuf
pip3 install -r requirements.txt
python3 main.py