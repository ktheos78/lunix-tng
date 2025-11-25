#!/bin/bash

# remove existing module, ignore error
rmmod lunix 2> /dev/null

# compile lunix
make

# insert lunix module into kernel
echo "Inserting Lunix module..."
insmod lunix.ko

# create lunix /dev/ files
echo "Creating Lunix /dev/ files..."
./mk-lunix-devs.sh

# attach lunix line discipline to ttyS1
echo "Attaching Lunix line discipline to ttyS1..."
./lunix-attach /dev/ttyS1
