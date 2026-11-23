#!/usr/bin/env bash
set -eu
ps -eo pid,ppid,stat,etimes,pcpu,rss,args | awk '/cc1plus|ninja|ab-benchmark-v9\/tools/ && !/awk/ { print substr($0,1,240) }'
free -m
