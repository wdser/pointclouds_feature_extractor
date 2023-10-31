# !/bin/bash
function main()
{
ACTION="$1"
if [ "check" == "$ACTION" ]; then
    echo "check"
	mkdir -p build && cd build/ && cmake .. && make -j 16
elif [ "build" == "$ACTION" ]; then
    echo "build"
	mkdir -p build && cd build/ && rm -rf ./* && cmake .. && make -j 16
elif [ "stop" == "$ACTION" ]; then
    echo "stop"
	kill $(ps -ef | grep feature_extractor | grep -v grep | awk '{print $2}')
else
    cd build && ./feature_extractor
fi
}

main $1

