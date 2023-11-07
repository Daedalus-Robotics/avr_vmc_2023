#!/usr/bin/env bash

sudo cp "$(dirname "$0")/vmc.service" /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable vmc.launch
sudo systemctl start vmc.launch