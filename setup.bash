#!/bin/bash

# project meta-data

WS_NAME="rovi_ws"
WS_PATH="$PWD/$WS_NAME"
PKG_NAME="rovi_system"
PROJ_NAME="ROVI"
GIT_URI="git@github.com:martinandrovich/rovi_system.git" # preferably SSH
DEMO_CMD="roslaunch rovi_system demo.launch"

# check dependencies
if ! hash rosdep vcs &> /dev/null
then
	echo "Unmet dependecies. Please make sure that rosdep and vsctool are installed."
	return
	exit
fi

# install apt dependencies (vcstool and rosdep)
# sudo apt install git python3-vcstool python3-rosdep -y

# confirm installation
read -p "This script will create the catkin workspace at '$WS_PATH' and install $PROJ_NAME. Continue? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then return;exit; fi

# setup workspace
echo -e  "\n\e[104mCreating catkin workspace...\e[49m\n" && sleep 1
mkdir -p $WS_PATH/src
cd $WS_PATH
catkin init

# clone project package
echo -e  "\n\e[104mCloning project package...\e[49m\n" && sleep 1
git clone $GIT_URI $WS_PATH/src/$PKG_NAME
# REPOS_FILE_PATH=$WS_PATH/src/$PKG_NAME/$PKG_NAME.repos
# REPOS_FILE_PATH="$GIT_URI/$PKG_NAME.repos"
# REPOS_FILE_PATH="$GIT_URI/.repos"
REPOS_FILE_PATH=$WS_PATH/src/$PKG_NAME/.repos

# clone packages (vcstool)
echo -e  "\n\e[104mCloning packages...\e[49m\n" && sleep 1
cd $WS_PATH
vcs import < $REPOS_FILE_PATH
vcs pull src

# update and install any system dependencies (rosdep)
echo -e  "\n\e[104mInstalling dependencies...\e[49m\n" && sleep 1
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build the workspace
echo -e  "\n\e[104mBuilding workspace...\e[49m\n" && sleep 1
catkin build

# add alias to .bashrc
echo -e "\n# $PROJ_NAME\nalias $WS_NAME='source $WS_PATH/devel/setup.bash && cd $WS_PATH'" >> ~/.bashrc
source ~/.bashrc

# delete script
cd .. && rm setup-project.bash

# finish
echo -e  "\n\e[104mInstallation complete!\e[49m\n"
echo -e "The project has been installed.\n\nSource the workspace and run '$DEMO_CMD' to verify the installation. Run '$WS_NAME' in any terminal to automatically configure the workspace and navigate to its directory.\n"
