#!/bin/bash

for file in `dir -d *.pcd` ; do
echo "processing $file"
rosrun cfg3d makeGraph  $file > out.$file
done

