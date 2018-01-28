#!/bin/sh

change_ssh(){
	sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
	service sshd restart
}

export PATH=${VM_PATH}

username="carndekf"
password="password123"
pass=$(perl -e 'print crypt($ARGV[0], "password")' $password)
useradd -m -p $pass $username >/dev/null 2>&1
[ $? -eq 0 ] && echo "User has been added to system!" || echo "User creation failed"
usermod -aG sudo $username

export DEBIAN_FRONTEND=noninteractive
apt-get update

chown -R udgcc ${PATH}
chmod -R 775 ${PATH}

apt-get install -y git

cd ${PATH}
git clone https://github.com/udacity/CarND-Extended-Kalman-Filter-Project.git carnd_ekf
cd carnd_ekf
chmod 555 ./install_ubuntu.sh
./install_ubuntu.sh