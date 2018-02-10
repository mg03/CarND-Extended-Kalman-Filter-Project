#!/bin/sh

export VMPATH="/opt/cpp_workspace"

chown -R vagrant ${VMPATH}
chmod -R 775 ${VMPATH}

cd ${VMPATH}
git clone https://github.com/udacity/CarND-Extended-Kalman-Filter-Project.git carnd_ekf
cd carnd_ekf
chmod 555 ./install_ubuntu.sh
./install_ubuntu.sh