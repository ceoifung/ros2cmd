#!/bin/bash
# xrapp的环境变量
home_directory=$(echo ~)
ros_workspace=colcon_ws

ros2_install_dir="$home_directory/$ros_workspace" # 将此路径替换为您的 ROS 2 安装路径
# package_names=$(find "$ros2_install_dir" -name "package.xml" -exec dirname {} \; | xargs -I {} basename {})
# readarray -t package_array <<< "$package_names"
# 动态根据工作空间获取所有的packages
get_dynamic_pkg_options(){
    local ament_prefix_path
    # 获取 AMENT_PREFIX_PATH 环境变量的值
    ament_prefix_path=$AMENT_PREFIX_PATH

    # 将路径以分号分隔为一个数组
    IFS=':' read -ra paths <<< "$ament_prefix_path"

    # 声明一个空数组用于保存符合条件的 package 名称
    package_names=()

    # 遍历路径数组
    for path in "${paths[@]}"; do
        # 检查路径是否包含 "colcon_ws"
        if [[ "$path" == *"${ros_workspace}"* ]]; then
            # 提取路径中的最后一个字符串
            package_name=$(basename "$path")

            # 将 package 名称添加到数组中
            package_names+=("$package_name")
        fi
    done
    echo "${package_names[@]}"  
}
package_array=$(get_dynamic_pkg_options)

# 获取launch文件
get_dynamic_launch_options()
{
    # local prev_param="$1"
    # local options=""
    # source_path=$(find $ros2_install_dir/src -type d -path "*${prev_param}")
    # # 查找launch的路径
    # launch_path=$(find $source_path/ -type d -path "*launch")
    # launch_files=$(find "$launch_path" -type f -exec basename {} \;)
    # readarray -t launch_array <<< "$launch_files"
    # echo "${launch_array[@]}"
    local package_name=$1
    executable_files=$(ls $ros2_install_dir/install/$package_name/share/$package_name/launch)
    readarray -t executable_array <<< "$executable_files"
    echo "${executable_array[@]}" 
}

# ros2 run的子菜单补全
get_dynamic_install_options(){
    local package_name=$1
    executable_files=$(ls $ros2_install_dir/install/$package_name/lib/$package_name)
    readarray -t executable_array <<< "$executable_files"
    echo "${executable_array[@]}" 
}

_ros2launch_complete()
{
    local cur prev options

    # 当前输入的参数
    cur="${COMP_WORDS[COMP_CWORD]}"
    # 上一个输入的参数
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    # 动态获取补全选项
    # for
    for opt in "${package_array[@]}"; do
    if [[ "$prev" == $opt ]]; then
        # 根据当前选项生成补全选项
        # 这里可以根据具体的逻辑来生成补全选项
             options=$(get_dynamic_launch_options "$prev")
            COMPREPLY=( $(compgen -W "${options[*]}" -- "$cur") )
            return 0
        fi
    done

    # 使用 compgen 生成补全建议
    COMPREPLY=( $(compgen -W "${package_array[*]}" -- "$cur") )
    return 0
}


# ros2 run的自动补全
_ros2run_complete()
{
    local cur prev options
    # 当前输入的参数
    cur="${COMP_WORDS[COMP_CWORD]}"
    # 上一个输入的参数
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    for opt in "${package_array[@]}"; do
    if [[ "$prev" == $opt ]]; then
            options=$(get_dynamic_install_options "$prev")
            COMPREPLY=( $(compgen -W "${options[*]}" -- "$cur") )
            return 0
        fi
    done
    # 使用 compgen 生成补全建议
    COMPREPLY=( $(compgen -W "${package_array[*]}" -- "$cur") )
    return 0
}


GREEN='\033[0;32m'
NC='\033[0m'
YELLOW='\033[0;33m'
ERROR='\033[0;31m'

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
        # 获取传入的参数
            args=("$@")
            num_args=${#args[@]}
            # 构建packages-select参数
            packages_select=""
            for ((i=0; i<$num_args; i++))
            do
                if [ $i -eq $((num_args-1)) ]; then
                    # 最后一个参数后面不添加逗号
                    packages_select+=" ${args[$i]}"
                else
                    packages_select+=" ${args[$i]} "
                fi
            done
            echo -e "${YELLOW}running colcon build --packages-select ${packages_select}...${NC}"
            colcon build --packages-select $packages_select
        else
            echo -e "${YELLOW}runing colcon build...${NC}"
            colcon build
        fi
    fi
}

_kill_with_name(){
    pidnum=`ps -ef|grep $1 | grep -v grep|awk '{print $2}'`
    if [ "$pidnum" != "" ];then
        for i in $pidnum
        do
            echo "start kill -9 $i"
            kill -9 $i
        done
    fi
}

killros(){
    # 杀死所有ROS 2进程
    echo "start kill ros program, please wait a minute"
    ros2 run xrrobot_base stop_rosts
    ros_ver=$(printenv ROS_DISTRO)
    _kill_with_name $ros_ver

    _kill_with_name $ros_workspace

    echo "Done"
}

ros2kill(){
    if [ "$1" != "" ];then
        if [ "$1" == "list" ]; then
            # 获取ROS 2节点列表
            node_list=$(ros2 node list)

            # 将节点列表分割成数组，并去除前导斜杠
            IFS=$'\n' read -d '' -r -a nodes <<< "$node_list"

            # 去除节点名称中的前导斜杠
            for i in "${!nodes[@]}"
            do
                nodes[$i]=${nodes[$i]#"/"}
            done
            echo "---------------------------------------------------------------------------"
            echo -e "${YELLOW}current ros2 node list. You can use follow command to kill the node${NC}"
            echo -e "${GREEN}ros2kill <node-name>${NC}or just ${GREEN}ros2kill${NC} to kill all"
            echo "---------------------------------------------------------------------------"
            for node in "${nodes[@]}"
            do
                echo "$node"
            done
            complete -W "${nodes[*]} list" ros2kill

        else
            # 杀死所有ROS 2进程
            _kill_with_name $1
        fi

    else
        killros
    fi
}

ros2purge(){
    echo "rm -rf $home_directory/.ros/log"
    rm -rf $home_directory/.ros/log
    echo "rm -rf $home_directory/$ros_workspace/log"
    rm -rf $home_directory/$ros_workspace/log
}

ros2show(){
    if [ "$1" != "" ]; then
        ros2 $1 list
    else
        echo "${ERROR}Missing params: ros2show <arg>${NC}"
    fi
}

# ros2 run的自动补全
_ros2pkg_complete()
{
    local cur prev options
    # 当前输入的参数
    cur="${COMP_WORDS[COMP_CWORD]}"
    # 使用 compgen 生成补全建议
    COMPREPLY=( $(compgen -W "$(get_dynamic_pkg_options)" -- "$cur") )
    return 0
}

complete -F _ros2launch_complete ros2launch
complete -F _ros2run_complete ros2run
complete -F _ros2pkg_complete ros2cd
complete -F _ros2pkg_complete ros2build
complete -W "list" ros2kill
complete -W "topic node service interface pkg param component action" ros2show


export XRROS_COLCON_WS="$ros2_install_dir"
