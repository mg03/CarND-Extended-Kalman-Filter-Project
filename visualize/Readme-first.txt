Setup guide for jupyter using docker

docker-machine start
eval $(docker-machine env)
docker run -it --rm -p 8888:8888 -v `pwd`:/src udacity/carnd-term1-starter-kit

The browse to the jupyter notebook replacing locaalhost with the ouput of following command:

docker-machine env | grep DOCKER_HOST | cut -d':' -f2 | sed 's|\/|''|g'