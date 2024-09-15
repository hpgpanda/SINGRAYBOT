#!/bin/bash



# 运行 Python 脚本
cd /home/agile/localization_init_tranmap/src/scripts

#gnome-terminal -- bash -c "source /home/hms/.bashrc && echo Map_=${Map_} && sleep 5"
gnome-terminal -- bash -c "source /home/agile/.bashrc && python3 /home/agile/localization_init_tranmap/src/scripts/one-touch_start.py"

