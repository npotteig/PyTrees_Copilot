#!/bin/bash

# Get Command Line Args using POSIX style
OPTSTRING=":p:h"

while getopts ${OPTSTRING} opt; do
  case ${opt} in
    p)
      pkg_name="${OPTARG}"
      ;;
    h)
      echo "Usage: ./$(basename "$0") -p <package name>"
      exit 1
      ;;
    :)
      echo "Option -${OPTARG} requires an argument."
      exit 1
      ;;
    ?)
      echo "Invalid option: -${OPTARG}."
      exit 1
      ;;
   
  esac
done

# Check if all required arguments are provided
if [ -z "$pkg_name" ]; then
  echo "All arguments are required"
  echo "Usage: $(basename "$0") -p <package name>"
  exit 1
fi

echo "Package Name: $pkg_name"

source ./install/setup.sh

ros2 launch "$pkg_name" tree.launch.py pkg_name:="$pkg_name" emulate_tty:=true
