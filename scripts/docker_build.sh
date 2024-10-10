#!/bin/bash -e

tagname=13.3

docker build . -t leemagnusson/arm-gcc:$tagname
docker tag leemagnusson/arm-gcc:$tagname leemagnusson/arm-gcc:latest
docker push leemagnusson/arm-gcc:$tagname #leemagnusson/arm-gcc:latest
