#!/bin/bash

#------------------------------------------
#ATTENTION!! This script are not completed
#------------------------------------------
#Developed by Oleksii Klochko
#------------------------------------------

E_NOARGS=85
E_WRONGOPTION=86
E_WRONGPARAM=87
E_DIRCREATE=88
DEFAULT_PATH="$HOME/bash"
DEFAULT_NAME="task1.out"
PARAMETER_NUM="-n"
HELP_TEXT="This is help text"


COWSAY=$(which cowsay)
if [ -z $COWSAY ];then
	COMMAND="echo"
else
	COMMAND="cowsay"
fi

function HelpInfo {
	$COMMAND "$HELP_TEXT"
}


args=$#
lastarg=${!args}
if [ -z "$1" ]; then
	echo "There is no arguments!" >&2
	exit $E_NOARGS
fi

while [ -n "$1" ]
do
	case "$1" in
		-h) HelpInfo
				exit 0;;
		--help)
				HelpInfo
				exit 0;;
		-n) param="$2"
				str="*[0-9]"
				if [[ ! "$param" =~ ^[0-9]{1,4}$ ]]; then
					echo "Wrong [num] parameter!" >&2
					exit $E_WRONGPARAM
				else
					if [ ! "$param" ]; then
						echo "[num] should be greather than 0" >&2
						exit $E_WRONGPARAM
					fi
				fi
				shift ;;
		*) echo "$1 is not an option" >&2
				exit $E_WRONGOPTION;;
	esac
	shift
done


DATE=$(date)
CPU=$(grep -m 1 "model name" /proc/cpuinfo)
POS=$(expr index "$CPU" :)
CPU=${CPU:$POS}

MEM=$(grep -m 1 "MemTotal" /proc/meminfo)
POS=$(expr index "$MEM" :)
MEM=${MEM:$POS}
MEM=${MEM%[kK][bB]}
MEM=$((MEM>>10))

MTH_VEND=$(dmidecode -s baseboard-manufacturer)
MTH_NAME=$(dmidecode -s baseboard-product-name)
SSN=$(dmidecode -s system-serial-number)

OS=$(grep -m 1 "NAME" /etc/os-release)
OS=${OS#*=}
OS=${OS%*\"}
OS_VER=$(grep -m 1 "VERSION" /etc/os-release)
OS_VER=${OS_VER#*=}
OS_VER=${OS_VER#*\"}
KERNEL=$(uname -r)
HW_TYPE=$(uname -m)
INSTALL_DATE=$(uname -v)
HOSTMANE=$(uname -n)
UPTIME=$(uptime -p)
PROCESSES=$(ps -A --no-headers | wc -l)
USERS=$(w -h | wc -l)


#  TODO: file checker should be implemented
FILE_PATH=$DEFAULT_PATH

if [ -d "$FILE_PATH" ]; then
	if [ `mkdir $FILE_PATH` ]; then
		echo "Can't create directory at $FILE_PATH" >&2
		exit $E_DIRCREATE
	fi
fi
cd $FILE_PATH
exec 1>$DEFAULT_NAME

echo "Date: ""$DATE"
echo "---- Hardware ----"
echo "CPU: " $CPU
echo "RAM: " $MEM "MB"
echo "Motherboard: \""$MTH_VEND"\", \""$MTH_NAME"\""
echo "System Serial Number: " $SSN
echo "---- System ----"
echo "OS Distribution: ""$OS" "$OS_VER"
echo "Kernel version: "$KERNEL $HW_TYPE
echo "Installation date: "$INSTALL_DATE
echo "Hostname: "$HOSTNAME
echo "Uptime: "$UPTIME
echo "Processes running: "$PROCESSES
echo "User logged in: "$USERS
echo "---- Network ----"


NUM_OF_INT=`ip a|grep  ^[0-9]:[[:space:]]*|wc -l`
#TODO: create array for collecting data 
for i in `seq $NUM_OF_INT`; do
	INTERFACE=$(ip a|grep  ^[0-9]:[[:space:]]*|grep $i:| awk '{ print $2}')
	#INTERFACE=$(ip a|awk '{print $1 $2}'|grep -m 1 $i:)
	#INTERFACE=${INTERFACE#$i:}
	IP_ADD=$(ip addr show dev $INTERFACE | grep "inet " | awk '{ print $2 }')
	if [ -z $IP_ADD ]; then
		echo "$INTERFACE" "Unknown"
	else
		echo "$INTERFACE" "$IP_ADD"
	fi
done
echo "----\"EOF\"----"

exit 0
