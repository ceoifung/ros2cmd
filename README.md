# ros2cmd
An efficient ros2 command tool, with which you can use commands like ros1, such as ros2launch. ros2cd , ro2run, ros2build, ros2kill

It'll be faster than the ros2 command autocomplete and association and more concise.

**Note: All commands are valid only in the `~/colcon_ws` workspace, You can change linux environment value: XRROS_COLCON_WS to update, see how to use**

## how to install
> There are two ways to use this tools

### first way
just download [setup.bash](./setup.bash), and put it in the system. And then add follow command in `~/.bashrc`

```shell
vim ~/.bashrc

# add new line 
source <where you put setup.bash>
```

After installing, you already can use it

### second way
You also can build a linux deb package, and then install it
ros2cmd
```shell
git clone https://github.com/ceoifung/ros2cmd
cd ros2cmd
chmod 777 ./make.sh
./make.sh
```

when you run `make.sh`, you will see os arch list, select the right arch. If you don't kown your or arch, run `uname -a`

```shell
# current support two arch
1) arm64
2) amd64
#?
```

Finally, install
```shell
sudo dpkg -i <deb-package>

# Add setup.bash
vim ~/.bashrc

# add new line 
source <where you put setup.bash>
```

## how to use
> Note: The following commands will only associate the specified workspace `~/colcon_ws`, if you want to change it then you need to modify `XRROS_COLCON_WS`, command: `export XRROS_COLCON_WS=your ros2 workspace path`.

In order to simplify the execution of commands in ros2, the following commands have been implemented, and the following commands can all be set by pressing the <strong style="color:red">TAB</strong> key for complementary settings

### ros2cd
> Open the ros2 package source directory, like ros1's command `roscd`

```shell
ros2cd <package-name>
```

### ros2launch
> Launch the ros2 launch files, like ros1's `roslaunch <package-name> <launch-file>`

```shell
ros2launch <package-name> <launch-file>
```

### ros2build
> build the package, If it carries arguments, then it is equivalent to `colcon build --packages-select <package-name>`; otherwise, it is `colcon build`.

```shell
# build specified package
ros2build <package-name>
# or build all workspace package
ros2build
```

### ros2run
> run the ros2 node, like ros1's `rosrun`

```shell
# ros2 run <package-name> <node-name>
ros2run <package-name> <node-name>
```

### ros2list
> list some ros2 parameters

```shell
ros2list <arg>
# supported parameters 
# action     component  param      service    
# interface  node       pkg        topic  
```

### ros2kill
> Violent termination of all ros2 nodes

- list current ros2 node
```shell
ros2kill list
```

- kill all ROS2 node
```shell
ros2kill
```

- Violent kill ros2 node
```shell
ros2kill <node-name>
``` 

### ros2clean
> Remove ros2 build log file and runtime logs

```shell
ros2clean
```

---
Powered by [ceoifung](https://github.com/ceoifung)
