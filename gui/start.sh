#!/bin/bash
# 获取脚本的完整路径
script_path=$(dirname "$0")
# 如果脚本是以相对路径运行的，那么需要转换为绝对路径
absolute_script_path=$(cd "$script_path" && pwd)
echo "脚本所在目录: $absolute_script_path"

# 运行 Python 脚本
cd ${absolute_script_path}
python3 ${absolute_script_path}/gui.py
