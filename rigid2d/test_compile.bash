#!/bin/bash
#A bash script to pass inputs to the tranform file
g++ -Wall -Wextra -g -std=c++17 -o rigid2d_test main.cpp rigid2d.cpp
rm out.txt
chmod +x rigid2d_test
./rigid2d_test < test1_input.txt >> out.txt

cmp -s out.txt test1_answer.txt && printf "Success \n" 
cmp -s out.txt test1_answer.txt || printf "Failure \n" 
