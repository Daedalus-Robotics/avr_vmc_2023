#!/usr/bin/env bash

mkdir /home/daedalus/.ros || true
cp "$(dirname "$0")/../config/csi.yaml" /home/daedalus/.ros/csi.yaml

sudo cp "$(dirname "$0")/vmc.service" /etc/systemd/system/
sudo cp "$(dirname "$0")/nvargus-daemon.service" /etc/systemd/system/

sudo systemctl daemon-reload

sudo systemctl enable nvargus-daemon.service
sudo systemctl restart nvargus-daemon.service

sudo systemctl enable vmc.service
sudo systemctl restart vmc.service