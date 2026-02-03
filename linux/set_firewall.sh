#!/bin/bash

# Search device IP and add it to the firewall allowlist
script_dir=$(cd $(dirname $0);pwd) 
ARCH=$(uname -m)
echo "arch:" $ARCH
if [ "$(echo $ARCH | grep "aarch")" != "" ]; then
	search_tool="$script_dir/Tool/arm_x64/lx_boardcast_tool"
elif [ "$(echo $ARCH | grep "x86_64")" != "" ]; then
	search_tool="$script_dir/Tool/linux_x64/lx_boardcast_tool"
else 
	echo "not found matched platform"
	exit
fi

echo $search_tool
if [ -e $search_tool ]; then
	echo "found search tool success, path:$search_tool"
	
	echo "found device ip:$devip"
else
	echo "not found search tool"
fi


# Detect which firewall is running
firewall_status=$(systemctl status ufw | grep Active)
if [[ $firewall_status == *"active (running)"* ]]; then
	echo "ufw is running."
	sudo ufw allow 9700
	sudo ufw allow 9800
	sudo ufw allow 9900
	sudo ufw allow 3956
	sudo ufw allow 3959
	sudo ufw allow 31900
	sudo ufw allow 32000
	sudo ufw allow 39560
	
	devip=$(lx_boardcast_tool | grep "search_device_ip:" | awk '{print $2}')
	sudo ufw allow from $(devip) to any
	sudo ufw reload
else
	echo "ufw is not running, do nothing."
fi


firewall_status=$(systemctl status firewalld | grep Active)
if [[ $firewall_status == *"active (running)"* ]]; then
	echo "firewalld is running."
	sudo firewall-cmd --permanent --zone=public --add-port=$9700/udp --add-port=$9800/udp --add-port=$3956/udp --add-port=$3959/udp 
	--add-port=$31900/udp --add-port=$32000/udp --add-port=$39560/udp  --add-port=$9900/tcp
	devip=$(lx_boardcast_tool | grep "search_device_ip:" | awk '{print $2}')
	sudo firewall-cmd --permanent --zone=public --add-source=$devip
	sudo firewall-cmd --reload
else
	echo "firewalld is not running, do nothing."
fi


firewall_status=$(systemctl status iptables | grep Active)
if [[ $firewall_status == *"active (running)"* ]]; then
	echo "iptables is running."
	sudo iptables -A INPUT -p udp -m multiport --dports 9700,9800,3956,3959,31900,32000,39560 -j ACCEPT
	sudo iptables -A INPUT -p tcp --dport 9900 -j ACCEPT
	devip=$(lx_boardcast_tool | grep "search_device_ip:" | awk '{print $2}')
	sudo iptables -A INPUT -p tcp --src $devip -j ACCEPT
	sudo iptables -A INPUT -p udp --src $devip -j ACCEPT
	sudo iptables-save > /etc/iptables/lx_rules.v4
	sudo iptables-restore < /etc/iptables/lx_rules.v4
	sudo systemctl restart iptables 
else
	echo "iptables is not running, do nothing."
fi














