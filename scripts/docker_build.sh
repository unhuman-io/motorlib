#!/bin/bash -e

docker build . -t leemagnusson/arm-gcc:latest
docker push leemagnusson/arm-gcc:latest