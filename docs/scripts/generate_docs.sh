#!/usr/bin/env bash
# Set the ROS workspace to a default value if not set
if [[ -z "${ROS_WORKSPACE}" ]]; then
  ROS_WORKSPACE=${HOME}/catkin_ws
fi

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source ${ROS_WORKSPACE}/devel/setup.bash

# Get the list of packages to skip during unit testing
ignore_packages_file='ignore_packages.txt'
ignore_packages=()

# Read the packages to ignore to a list
while read line; do
    # reading each line
    ignore_packages+=($line)
done < $ignore_packages_file

# Save the current directory
curr_dir=$(pwd)

# Get the catkin workspace absolute directory
workspace_abs_path=$(roscd && cd src && pwd)

# Get the documentation directory (full path)
documentation_directory=${workspace_abs_path}/documentation/docs

# Check if the directory exists, otherwise create it
if ! [[ -d documentation_directory ]]; then
    mkdir -p ${documentation_directory}
fi

echo ${documentation_directory}

# Iterate through list of ROS packages in the current workspace
for package in $(catkin list); do

    # Check if the package is not in the ignore list
    if ! [[ ${ignore_packages[*]} =~ (^|[[:space:]])"${package}"($|[[:space:]]) ]] && ! [[ '-' =~ (^|[[:space:]])"${package}"($|[[:space:]]) ]]; then

        # Go to the ROS package
        roscd ${package}

        # Get the package absolute path
        package_abs_path=$(pwd)

        # Get the package path relative to the workspace directory
        package_path=${package_abs_path//$workspace_abs_path}

        # Remove the medusa_base if still in the path
        package_path=${package_path//\/medusa_base}

        # Check if the package has a doc directory
        if [[ -d "doc" ]] ; then
            # Create a symbolic link for the documentation folders inside the 
            mkdir -p ${documentation_directory}${package_path}
            ln -s "${package_abs_path}/doc" "${documentation_directory}${package_path}" --force
        fi

        # Check if the package has a doc directory
        if [[ -d "docs" ]] ; then
            # Create a symbolic link for the documentation folders inside the 
            mkdir -p ${documentation_directory}${package_path}
            ln -s "${package_abs_path}/docs" "${documentation_directory}${package_path}" --force
        fiS
    fi
done

# Go back to the original directory
cd ${curr_dir}

# Run mkdocs to build the documentation
mkdocs build