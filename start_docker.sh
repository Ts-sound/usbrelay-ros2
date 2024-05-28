docker run -d \
 --privileged \
 -p 2202:22 \
 -v .:/opt/ros2_ws \
 usbrelay:latest \
 " /bin/bash -c "sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/'