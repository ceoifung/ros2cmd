#!/bin/bash

# Color variables
GREEN='\033[0;32m'
NC='\033[0m'
YELLOW='\033[0;33m'
ERROR='\033[0;31m'

# find yourself workspace ros2 packages
home_directory=$(echo ~)
ros_workspace=colcon_ws
# Your own ros2 workspace path
ros2_install_dir="$home_directory/$ros_workspace"
package_names=$(find "$ros2_install_dir" -name "package.xml" -exec dirname {} \; | xargs -I {} basename {})
readarray -t package_array <<< "$package_names"

###########################

# get ros2 packages launch files
get_dynamic_launch_options()
{
    local package_name=$1
    executable_files=$(ls $ros2_install_dir/install/$package_name/share/$package_name/launch)
    readarray -t executable_array <<< "$executable_files"
    echo "${executable_array[@]}" 
}

# get ros2 packages executable node
get_dynamic_install_options(){
    local package_name=$1
    executable_files=$(ls $ros2_install_dir/install/$package_name/lib/$package_name)
    readarray -t executable_array <<< "$executable_files"
    echo "${executable_array[@]}" 
}

# ros2launch auto complete function
_ros2launch_complete()
{
    local cur prev options
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    for opt in "${package_array[@]}"; do
    if [[ "$prev" == $opt ]]; then
            options=$(get_dynamic_launch_options "$prev")
            COMPREPLY=( $(compgen -W "${options[*]}" -- "$cur") )
            return 0
        fi
    done

    COMPREPLY=( $(compgen -W "${package_array[*]}" -- "$cur") )
    return 0
}


# ros2run auto complete function
_ros2run_complete()
{
    local cur prev options
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    for opt in "${package_array[@]}"; do
    if [[ "$prev" == $opt ]]; then
            options=$(get_dynamic_install_options "$prev")
            COMPREPLY=( $(compgen -W "${options[*]}" -- "$cur") )
            return 0
        fi
    done
    COMPREPLY=( $(compgen -W "${package_array[*]}" -- "$cur") )
    return 0
}

ros2cd(){
    if [ "$1" != "" ];then
        source_path=$(find $XRROS_COLCON_WS/src -type d -path "*$1")

        readarray -t path_array <<< "$source_path"

        # 假设有一个名为array的数组
        if [ -z "${path_array[*]}" ]; then
            echo -e "${ERROR}There is no such path!${NC}"
        else
            cd "${path_array[0]}"
        fi
    else
        cd $XRROS_COLCON_WS
    fi
}

ros2launch(){
    if [ "$1" != "" ];then
        ros2 launch $1 $2
    fi
}

ros2run(){
    if [ "$1" != "" ];then
        ros2 run $1 $2
    fi
}

ros2build(){
    cur_ws=$(pwd)
    if [ "$cur_ws" != "$XRROS_COLCON_WS" ];then
        echo -e "${YELLOW}You should go to ros2 workspace[$XRROS_COLCON_WS] to build package, use ros2cd to quick switch directory${NC}"
    else
        if [ "$1" != "" ];then
            colcon build --package-select $1
        else
            colcon build
        fi
    fi
}

ros2kill(){
    # 杀死所有ROS 2进程
    local ros_ver
    ros_ver=$(printenv ROS_DISTRO)
    pkill -f "$ros_ver"
    pkill -f "$ros_workspace"
}

complete -F _ros2launch_complete ros2launch
complete -F _ros2run_complete ros2run
complete -W "${package_array[*]}" ros2cd
complete -W "${package_array[*]}" ros2build
# export yourself workspace
export XRROS_COLCON_WS="$ros2_install_dir"
