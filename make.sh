#! /bin/bash
###
 # @Author: Ceoifung
 # @Date: 2023-07-01 09:26:52
 # @LastEditors: Ceoifung
 # @LastEditTime: 2024-01-11 19:37:18
 # @Description: XiaoRGEEK All Rights Reserved. Copyright © 2023
###
productName="ros2cmd"
output_dir=$productName
arch_dir="$output_dir/DEBIAN"
# bin_dir="$output_dir/usr/bin"
install_dir=$output_dir/opt/$productName/

apkVer=1.0
# build dkg arch
buildLists=(
    "arm64"
    "amd64"
)

echo -e "\033[35mSelect packages deb architecture(Default:arm64):\033[0m"
select build in "${buildLists[@]}"
do
    if [ "$build" = "" ]; then
        build="arm64"
    fi
    pkgArch=$build
    echo "Your Choice: $pkgArch"
    break;
done

# 生成输出文件
if [ ! -d "$arch_dir" ]; then
    echo -e "\033[35mDEBIAN dir: $arch_dir\033[0m"
    mkdir $arch_dir -p
else
    rm -rf $arch_dir/*
fi
# if [ ! -d "$bin_dir" ]; then
#     echo -e "\033[35mBIN dir: $bin_dir\033[0m"
#     mkdir $bin_dir -p
# else
#     rm -rf $bin_dir/*
# fi
if [ ! -d "$install_dir" ]; then
    echo -e "\033[35mInstall dir: $install_dir\033[0m"
    mkdir $install_dir -p
else
    rm -rf $install_dir/*
fi


# 写入打包参数
cat>${arch_dir}/control<<EOF
package: $productName
version: $apkVer
architecture: $pkgArch
maintainer: ceoifung
description: A useful ros2 command simplify tools, Copyright (c) 2023, Powered by Ceoifung.
EOF

echo "chmod to scripts"
cp ./setup.bash $install_dir/

dpkg -b $output_dir ./$productName-$pkgArch-$apkVer.deb
echo -e "\033[35mYou can use follow command to install deb-package:\033[0m\n\n\033[33msudo dpkg -i ./$productName-$pkgArch-$apkVer.deb \033[0m \n\n\033[44;37mAll done, thank you.\033[0m\n"


