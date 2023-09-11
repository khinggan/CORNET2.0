#!/bin/bash

docker build -t cornet:focalfoxyNWH -f Dockerfile.focalfoxyNWH .
# docker build -t cornet:ubuntu1804 -f Dockerfile.ubuntu1804 .
docker build -t commbot:khinggan -f Dockerfile.khinggan .