#!/bin/bash
echo "Start experiment"
. install/setup.bash
timeout 5s ros2 run experiments push_rot_gazebo

counter=50

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_rot_gazebo 0.05
    timeout 5s ros2 run experiments push_rot_main_fresh 0.05 wifi5_loaded 0.0004 1 0 0 #mode after step size->1: only communication delay, 2: communication & computation delay, 3: only computation delay,  else: no delay
    # Increment the value of n by 1
    (( counter++ ))
done

counter=1000

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_rot_gazebo 0.05
    timeout 5s ros2 run experiments push_rot_main_fresh 0.05 wifi5_loaded 0.0004 1 0 1 #mode after step size->1: only communication delay, 2: communication & computation delay, 3: only computation delay,  else: no delay
    # Increment the value of n by 1
    (( counter++ ))
done

counter=149

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_rot_gazebo 0.05
    timeout 5s ros2 run experiments push_rot_main_fresh 0.05 private5g_urllc 0.0004 1 0 #1: only communication delay, 2: communication & computation delay, 3: only computation delay,  else: no delay
    # Increment the value of n by 1
    (( counter++ ))
done

counter=149

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_rot_gazebo 0.05
    timeout 5s ros2 run experiments push_rot_main_fresh 0.05 private4g 0.0004 1 0 #1: only communication delay, 2: communication & computation delay, 3: only computation delay,  else: no delay
    # Increment the value of n by 1
    (( counter++ ))
done

counter=149

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_rot_gazebo 0.05
    timeout 5s ros2 run experiments push_rot_main_fresh 0.05 wifi5_ideal 0.0004 1 0 #1: only communication delay, 2: communication & computation delay, 3: only computation delay,  else: no delay
    # Increment the value of n by 1
    (( counter++ ))
done

counter=189

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_rot_gazebo 0.05
    timeout 5s ros2 run experiments push_rot_main_fresh 0.05 private5g 0.0004 1 0 #1: only communication delay, 2: communication & computation delay, 3: only computation delay,  else: no delay
    # Increment the value of n by 1
    (( counter++ ))
done

counter=149

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 5s ros2 run experiments push_rot_gazebo 0.05
    timeout 5s ros2 run experiments push_rot_main_fresh 0.05 ethernet 0.0004 1 0 #1: only communication delay, 2: communication & computation delay, 3: only computation delay,  else: no delay
    # Increment the value of n by 1
    (( counter++ ))
done






echo "Finished experiment"
