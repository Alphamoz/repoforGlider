#!/bin/bash
eval $(xuartctl --server --port 4 --speed 115200 --mode 8n1 2>&1); ln -s $ttyname /dev/ttyxuart4
eval $(xuartctl --server --port 1 --speed 115200 --mode 8n1 2>&1); ln -s $ttyname /dev/ttyxuart1
pgrep -f xuart -l
