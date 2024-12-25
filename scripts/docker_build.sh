#!/bin/bash -e

tagname=14.2

docker build . -t leemagnusson/arm-gcc:$tagname
docker tag leemagnusson/arm-gcc:$tagname leemagnusson/arm-gcc:latest
docker push leemagnusson/arm-gcc:$tagname #leemagnusson/arm-gcc:latest
