#!/bin/bash

# Subprocess special case
if [[ "$1" == "$0" ]]; then

	if ! [[ $2 =~ Speed ]]; then
		echo "$2" >&2
	elif [[ "$(echo "$2" | awk '{ print $2 }')" -gt "0" ]]; then
		echo -n -e "`date -u "+%b-%d, %Y %H:%M:%S"`\t"
		echo -n -e "`date -u "+%Z"`\t"
		echo -n -e "`TZ='America/New_York' date "+%b-%d, %Y %H:%M:%S"`\t"
		echo -n -e "`TZ='America/New_York' date "+%Z"`\t"
		#echo "$2" | awk '{ print $2 "\t" $5 " @" $7 "°" }'
		actual=$(echo "$2" | awk '{ print $2 }')
		measured=$(echo "$2" | awk -F '\\(Measured | @ ' '{print $2}')
		angle=$(echo "$2" | awk -F ' @ | degrees offset\\)' '{print $2}')
		verified=$(echo "$2" | awk -F "verified at | %" '{print $2}')

		echo -n -e "${actual}\t"

		if [[ "$verified" != "" ]]; then
			echo -n -e "${verified}%"
		fi

		echo -n -e "\t"
		
		if [[ "$angle" != "" ]]; then
			echo -n -e "${measured} @${angle}°"
		fi

		echo ""
	fi
	exit
fi

# Main script
serialport="$1"
filepath="$2"

echo "serialport = $serialport"
echo "filepath   = $filepath"
echo ""

if [[ "$serialport" == "" || "$filepath" == "" ]]; then
	echo "Usage: speedgun_to_tsv.sh /dev/ttyXX outfile.tsv"
	exit 1
fi

# Set up port speeds
stty -F "$serialport" ispeed 115200 -echo -icrnl 

# Set up terminal tab spacing
tabs 5

# Print header
echo -e "Date         Time    \tTZ \tDate         Time    \tTZ \tSpd\tVfyd\tRaw" | tee -a "$filepath"

# Process data
#function testgen() { for i in {1..5}; do echo "Speed $i$i mph"; sleep 3; done; }
#cat "$serialport" | xargs -d '\n' -n1 stdbuf -oL "$0" "$0" 
#cat "$serialport" | stdbuf -oL grep "Speed" | xargs -d '\n' -n1 "$0" "$0" | tee -a "$filepath" 
cat "$serialport" | stdbuf -oL xargs -d '\n' -n1 "$0" "$0" | tee -a "$filepath" 

#testgen | xargs -d '\n' -n1 echo | stdbuf -oL awk -v datestamp="$(date -u "+%Y-%b-%d %H:%M:%S %Z")" '{ print datestamp "\t" $2 }' 
