#!/bin/bash


# 
# Remove all docker images and container
# 
pushd ../../../..
docker ps -a -q # list all docker container
docker_running=`(docker ps -a -q | wc -l)`
if [ $docker_running -gt 0 ] ; then 
  docker stop $(docker ps -a -q)
  docker rm $(docker ps -a -q)
fi
docker system prune -a -f
docker volume prune -f
docker images -a # list all docker images
# docker rmi -f $(docker images -a) # remove all docker images


