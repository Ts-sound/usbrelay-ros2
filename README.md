# usbrelay-ros2
usbrelay ros2 driver


# build docker 

```bash

docker build -t usbrelay .

```

# start docker

```bash
docker run -d  --privileged -p 2202:22 -v $(pwd):/root/ros2_ws usbrelay:latest 
```

docker start <id>
docker attach <id>

echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc