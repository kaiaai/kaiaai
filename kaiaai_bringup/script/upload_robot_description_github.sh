#!/bin/bash

# Clone the robot description (where the script resides) into a new one
#   ros2 run kaiaai_snoopy clone_robot_description.sh /ros_ws/src/kaiaai_fido
[ -z "$1" ] && { echo "Description path missing" ; exit 1; }
[ ! -d "$1" ] && { echo "Description path invalid" ; exit 1; }
[ -z "$2" ] && { echo "Github username missing" ; exit 1; }

cd $1
pwd
# Get folder name without path
desc_name=${PWD##*/}
desc_name=${desc_name:-/}
# echo $desc_name

repo="https://api.github.com/repos/$2/$desc_name"
git_arg="https://$2@github.com/$2/$desc_name/"

if curl -fsS $repo &> /dev/null; then
  echo "Uploading to $git_arg"
else
  echo "$git_args repo not found"
  exit 1
fi

# TODO check remote is empty

git init
git add .
git commit -m "First commit"
git branch -M main
git remote add origin "$git_arg"
git push -u origin main

# TODO update package.xml Github link?
