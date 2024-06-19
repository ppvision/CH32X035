#!/bin/bash

usb_pdev=0x01120000
firmware=""


if [ $# -lt 1 ]; then
    echo "需要至少1个参数"
    echo "用法: firmware"
    exit 1
fi

if [ $# -eq 2 ]; then
	usb_pdev=$1
	firmware=$2
fi

if [ $# -eq 1 ]; then
    firmware=$1
fi

script_path=$(dirname "$0")

# my dev : 

WCHISPTool_CMD -p $usb_pdev -c $script_path/CH32V035.INIT -o program -v boot -f $firmware 
