#!/bin/sh
  
OS_NAME=`uname --`
if [ "$OS_NAME" = "Linux" ]; then
  USER_ID=`id -u`
  # Check required priviledge
  if [ "$USER_ID" != "0" ]; then
 	echo "This script can only be run by root user or sudoer"
    exit 1
  fi
fi

if [ "$1" = "10" ]; then
    SIZE=10485760 
elif [ "$1" = "15" ]; then
    SIZE=15728640 
elif [ "$1" = "20" ]; then
    SIZE=20971520 
elif [ "$1" = "25" ]; then
    SIZE=26214400 
else
    echo "use default mem_max 10MB."
    SIZE=10485760
fi

echo "Setting socket maximum buffer size to $SIZE"

if [ "$OS_NAME" = "Linux" ]; then

  PROC_ROOT=/proc/sys/net/core

  if [ ! -w $PROC_ROOT/wmem_max ]; then
    echo "Cannot write to $PROC_ROOT/wmem_max"
    exit 1
  fi

  if [ ! -w $PROC_ROOT/rmem_max ]; then
    echo "Cannot write to $PROC_ROOT/rmem_max"
    exit 1
  fi

  echo $SIZE > $PROC_ROOT/wmem_max
  echo $SIZE > $PROC_ROOT/rmem_max

  # If net.core.rmem_max present in /etc/sysctl.conf, change value to "$SIZE"
  #	else, add "net.core.rmem_max = $SIZE" to /etc/sysctl.conf
  if grep --silent ^net.core.rmem_max /etc/sysctl.conf; then
    sed -i 's/^net.core.rmem_max.*/net.core.rmem_max = '$SIZE'/g' /etc/sysctl.conf
  else
    echo "" >> /etc/sysctl.conf
    echo "net.core.rmem_max = $SIZE" >> /etc/sysctl.conf
  fi
  
  if grep --silent ^net.core.wmem_max /etc/sysctl.conf; then
    sed -i 's/^net.core.wmem_max.*/net.core.wmem_max = '$SIZE'/g' /etc/sysctl.conf
  else
    echo "" >> /etc/sysctl.conf
    echo "net.core.wmem_max = $SIZE" >> /etc/sysctl.conf
  fi
  
  exit 0
fi

echo "Error: unknown platform: $OS_NAME"
exit 1

