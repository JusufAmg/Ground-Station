# Ground-Station

Lightweight MAVLink ground station for monitoring and controlling drones over UDP.

## Features
- Telemetry receiver
- Live flight data
- Map view
- Mode commands

## Requirements
- Python 3.8+
- PySide6
- pymavlink
- requests

## Install
git clone <repo>
cd <repo>
pip install -r requirements.txt

## Run
python main.py
python simple_command_sender.py

## Structure
main.py
simple_command_sender.py
