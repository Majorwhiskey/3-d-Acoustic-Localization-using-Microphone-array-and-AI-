#!/bin/bash
# Script to run TDOA triangulation with virtual environment

# Activate virtual environment
source /home/head-node-5/project/venv/bin/activate

# Run the TDOA triangulation script
python3 tdoa_triangulation.py

# Deactivate virtual environment
deactivate

