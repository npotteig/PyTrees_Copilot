#!/bin/bash

# Get Command Line Args using POSIX style
OPTSTRING=":p:m:h"

while getopts ${OPTSTRING} opt; do
  case ${opt} in
    p)
      pkg_name="${OPTARG}"
      ;;
    m)
      monitor_file="${OPTARG}"
      ;;
    h)
      echo "Usage: ./$(basename "$0") -p <package name> -m <copilot monitor path>"
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
if [ -z "$pkg_name" ] || [ -z "$monitor_file" ]; then
  echo "All arguments are required"
  echo "Usage: $(basename "$0") -p <package name> -m <copilot monitor path>"
  exit 1
fi

echo "Package Name: $pkg_name"
echo "Monitor File: $monitor_file"

# Generate C99 Monitors from Haskell
runhaskell "$monitor_file"

# Create package
ros2 pkg create "$pkg_name" --build-type ament_cmake

# Copy monitor files
mv monitor_types.h monitor.h monitor.c "$pkg_name"/src/

# Generate python node directory
mkdir -p "$pkg_name"/"$pkg_name"/

# Copy python addon node files
cp file_generation/proj_addons/py_addons/* "$pkg_name"/"$pkg_name"/

# Copy Launch File
mkdir -p "$pkg_name"/launch
cp -r file_generation/proj_addons/launch/ "$pkg_name"/

# Copy CMakeLists.txt and package.xml
cp file_generation/proj_addons/build_updates/* "$pkg_name"/

# Replace package name in package.xml
sed -e "s|<!--PACKAGE_NAME-->|$pkg_name|g" "$pkg_name"/package.xml > "$pkg_name"/package_temp.xml
mv "$pkg_name"/package_temp.xml "$pkg_name"/package.xml

# Generate monitor ros node file
# Generate python bt node file
python3 file_generation/pipeline_script.py --header_file "$pkg_name"/src/monitor.h --cpp_dir "$pkg_name"/src/ --py_dir "$pkg_name"/"$pkg_name"/

# Make the python file executable
chmod +x "$pkg_name"/"$pkg_name"/bt_node.py
