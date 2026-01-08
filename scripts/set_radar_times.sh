#!/bin/bash

# Set up port speeds
stty -F /dev/ttyACM0 ispeed 115200 -echo -icrnl 

# Show output
cat /dev/ttyACM0 &
pid=$!

#echo 'C 43' > /dev/ttyACM0
#echo 'S 1000' > /dev/ttyACM0
#echo 'O 2000' > /dev/ttyACM0

# Behind transformer in front yard
echo 'C 39' > /dev/ttyACM0

#echo 'C 0' > /dev/ttyACM0
echo 'S 100' > /dev/ttyACM0
echo 'O 400' > /dev/ttyACM0
echo 'V 1' > /dev/ttyACM0

sleep 1
echo 'Waiting to check loop timing . . .'
sleep 5
echo 'E' > /dev/ttyACM0
sleep 1
echo 'E' > /dev/ttyACM0
sleep 1
echo 'E' > /dev/ttyACM0

echo 'Done'

kill $!
