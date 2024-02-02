eval $(xuartctl --server --port 4 --speed 115200 --mode 8n1 2>&1); ln -s $ttyname /dev/ttyS4
pgrep -f xuart -l

