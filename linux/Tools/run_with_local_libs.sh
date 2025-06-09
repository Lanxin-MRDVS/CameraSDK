#!/bin/bash
CURRENT_PATH=$(pwd)
if grep -q "export LD_LIBRARY_PATH=$CURRENT_PATH:\$LD_LIBRARY_PATH" ~/.bashrc; then
    echo "该路径已存在于LD_LIBRARY_PATH中"
else
    echo "export LD_LIBRARY_PATH=$CURRENT_PATH:\$LD_LIBRARY_PATH" >> ~/.bashrc
    echo "已添加当前路径到LD_LIBRARY_PATH"
    source ~/.bashrc
fi

