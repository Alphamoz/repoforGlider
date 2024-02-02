#!/bin/bash
eval $(xuartctl --server --port 4 --speed 9600 --mode 8n1 2>&1); ln -s $ttyname /dev/ttyxuart4
eval $(xuartctl --server --port 1 --speed 9600 --mode 8n1 2>&1); ln -s $ttyname /dev/ttyxuart1
eval $(xuartctl --server --port 0 --speed 9600 --mode 8n1 2>&1); ln -s $ttyname /dev/ttyxuart0
pgrep -f xuart -l
