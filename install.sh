#!/bin/bash

echo "LxCamera-MRDVS install program!"

# Return 0 for 'n', 1 for 'y'
ChooseYorN() 
{
    local q="$1 (Y/n) : "
    while true; do
        read -p "$q" yn
        case $yn in
            [Yy]* ) return 0; ;;
            [Nn]* ) return 1; ;;
            #* );;
        esac
    done
}

# Make sure script is only being run with root privileges
if [ "$(id -u)" != "0" ] ; 
then
    if ChooseYorN "You don't have root permission, Script will try to use sudo. Sure to continue?" ; 
    then
        SUDO="sudo "
        $SUDO echo ""
    else
        echo "***This script must be run with root privileges!***"
        echo "Script Exit..."
        exit 1;
    fi
fi


echo "move SDK to /opt/MRDVS"
DST_PATH="/opt/MRDVS"
$SUDO rm -rf /opt/Lanxin-MRDVS
$SUDO rm -rf ${DST_PATH}
$SUDO mkdir ${DST_PATH}
$SUDO cp -r ./SDK/include ${DST_PATH}
$SUDO mkdir ${DST_PATH}/lib

ARCH=$(uname -m)
echo "arch:" $ARCH
if [ "$(echo $ARCH | grep "arm")" != "" ]; then
	$SUDO cp -r ./SDK/lib/linux_arm32/* ${DST_PATH}/lib/
elif [ "$(echo $ARCH | grep "aarch")" != "" ]; then
	$SUDO cp -r ./SDK/lib/linux_aarch64/* ${DST_PATH}/lib/
elif [ "$(echo $ARCH | grep "x86_64")" != "" ]; then
	$SUDO cp -r ./SDK/lib/linux_x64/* ${DST_PATH}/lib/
else 
	echo "not found matched platform"
	exit
fi

echo ""
echo "export path: ${DST_PATH}/lib/"
export LD_LIBRARY_PATH=${DST_PATH}/lib/:$LD_LIBRARY_PATH

echo $SHELL
if [ $SHELL = '/bin/bash' ]; then
	
	BASH_FILE=~/.bashrc 
	if ! grep -Fq "/MRDVS/" $BASH_FILE; then
		sed -i '$a \export LD_LIBRARY_PATH='${DST_PATH}'/lib/:$LD_LIBRARY_PATH' $BASH_FILE
		source $BASH_FILE 
	fi
else
	BASH_FILE=~/.zshrc 
	if ! grep -Fq "/MRDVS/" $BASH_FILE; then
		sed -i '$a \export LD_LIBRARY_PATH=${DST_PATH}/lib/:$LD_LIBRARY_PATH' $BASH_FILE
		source $BASH_FILE 
	fi
fi

echo ""
echo "set socket buffer size"
$SUDO sh set_socket_buffer_size.sh

echo ""
echo "LxCamera-MRDVS install done!"
