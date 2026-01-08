#!/bin/bash

if [[ "$1" == "" ]]; then
    filename=`ls -tr *.tsv | tail -1`
else
    filename="$1"
fi

echo "Summary of file: $filename"

echo ""

# Print counts of each speed
echo "  Count MPH"
# Account for old vs new format
if egrep 'Vfyd' $filename > /dev/null; then
    #                           /-- cut out <50% Vrfd --\
		#                           |    note CTRL+V TAB    |
    summary=$(cat "$filename" | egrep -v '	[0-4]?[0-9]%' | awk '{print $9}' | egrep -v 'Raw|^$' | sort -n | uniq -c)
    echo -e "$summary"
elif egrep '[0-9], [0-9]' $filename > /dev/null; then
    summary=$(cat "$filename" | awk '{print $9}' | egrep -v 'Speed|^$' | sort -n | uniq -c)
    echo -e "$summary"
else
    summary=$(cat "$filename" | grep -v -P '\t0\t?' | awk '{print $7}' | egrep -v 'Speed|^$' | sort -n | uniq -c)
    echo -e "$summary"
fi

echo ""
echo "Computing statistics . . ."

if [[ "$2" == "" ]]; then
    lower_limit="0"
else
    lower_limit="$2"
    echo "Using lower limit $lower_limit MPH"
fi

if [[ "$3" == "" ]]; then
    upper_limit="999"
else
    upper_limit="$3"
    echo "Using upper limit $upper_limit MPH"
fi

echo ""

# Find min/max/avg
min=999
max=-1
sum=0
count=0
while read -r line; do
    spd_count=$(echo -n  "$line" | awk '{print $1}')
    speed=$(echo -n "$line" | awk '{print $2}')

    if [[ "$speed" -gt "0" && "$speed" -ge "$lower_limit" && "$speed" -le "$upper_limit" ]]; then
	# Process the value
	if [[ "$speed" -lt "$min" ]]; then
	    export min=$speed
	fi
	if [[ "$speed" -gt "$max" ]]; then
	    export max=$speed
	fi
	weighted_spd=$(( $speed * $spd_count ))
	sum=$(( $sum + $weighted_spd ))
	count=$(( $count + $spd_count ))
    fi
done <<< "$summary"

if [[ "$count" -gt "0" ]]; then
    avg=$(( $sum / $count ))

    echo "Min: $min"
    echo "Max: $max"
    echo "Avg: $avg"
    echo ""
    echo "Records analyzed: $count"
else
    echo "No speed entries detected."
fi

echo ""
