#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Missing command line argument"
    echo "Usage: $(basename $0) <command>"
    exit 1
fi

mkdir -p $USER_WS/persistence/$(id -u)/cache
mkdir -p $USER_WS/persistence/$(id -u)/ccache
mkdir -p $USER_WS/persistence/$(id -u)/install
mkdir -p $USER_WS/persistence/$(id -u)/build

rm -rf $HOME/.cache
rm -rf $HOME/.ccache

ln -s $USER_WS/persistence/$(id -u)/cache  $HOME/.cache
ln -s $USER_WS/persistence/$(id -u)/ccache  $HOME/.ccache
ln -s $USER_WS/persistence/$(id -u)/install $USER_WS/install
ln -s $USER_WS/persistence/$(id -u)/build   $USER_WS/build


# if bash_history is not a symlink or if it does not exist, remove it and create a new symlink
if [ ! -h "$HOME/.bash_history" ]; then
    if [ -f "$HOME/.bash_history" ]; then
        rm $HOME/.bash_history
    fi
    touch $USER_WS/persistence/$(id -u)/.bash_history
    ln -s $USER_WS/persistence/$(id -u)/.bash_history $HOME/.bash_history
fi

if [ ! -f "$USER_WS/persistence/$(id -u)/COLCON_IGNORE" ]; then
    touch $USER_WS/persistence/$(id -u)/COLCON_IGNORE
fi

. /opt/ros/$ROS_DISTRO/setup.bash

ros2 daemon start

echo
echo "ROS_DISTRO         : $ROS_DISTRO"
echo "ROS_DOMAIN_ID      : $ROS_DOMAIN_ID"
echo

# set the flag indicating that fixuid is done
touch fixuid_complete

exec "$@"
