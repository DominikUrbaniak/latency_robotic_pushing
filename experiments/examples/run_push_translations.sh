#!/bin/bash
echo "Start experiment"
. install/setup.bash
timeout 5s ros2 run experiments push_translation_gazebo

counter=100


while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 3s ros2 run experiments push_translation_gazebo 0.08
    timeout 5s ros2 run experiments push_translation_main 0.08 private4g 0.0006 60 125 1
    # Increment the value of n by 1
    (( counter++ ))
done

counter=100

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 3s ros2 run experiments push_translation_gazebo 0.08
    timeout 5s ros2 run experiments push_translation_main 0.08 private5g 0.00015 60 500 1
    # Increment the value of n by 1
    (( counter++ ))
done

counter=100

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 3s ros2 run experiments push_translation_gazebo 0.08
    timeout 5s ros2 run experiments push_translation_main 0.08 private5g_urllc 0.0003 60 500 1
    # Increment the value of n by 1
    (( counter++ ))
done

counter=100

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 3s ros2 run experiments push_translation_gazebo 0.08
    timeout 5s ros2 run experiments push_translation_main 0.08 ethernet 0.0003 60 125 1
    # Increment the value of n by 1
    (( counter++ ))
done

counter=100

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 3s ros2 run experiments push_translation_gazebo 0.08
    timeout 5s ros2 run experiments push_translation_main 0.08 wifi5_loaded 0.0003 60 125 1
    # Increment the value of n by 1
    (( counter++ ))
done

counter=100

while [ $counter -le 100 ]
do
    # Print the value of n in each iteration
    echo "Running $counter time"
    timeout 3s ros2 run experiments push_translation_gazebo 0.08
    timeout 5s ros2 run experiments push_translation_main 0.08 wifi5_ideal 0.00015 250 500 1
    # Increment the value of n by 1
    (( counter++ ))
done



echo "Finished experiment"
