Setup for using VirtualBox to code and running sim on mac book 

Install VirtualBox and Vagrant

set env vars on Mac OSX
VM_PATH and LOCAL_PATH

export LOCAL_PATH="/Users/cpp_workspace"
export VM_PATH="/opt/cpp_workspace"

These are used to sync the folders from Local Machine to Remote Machine. This is necessary so
that we can use sublime or any other ide on Mac book and run the build command on the linux guest

Run `vagrant up`

In order to ssh in the machine

ssh vagrant@127.0.0.1 -p 32222
Provide password:   vagrant   
You can change it before provisioning by changing it in the install_stuff.sh file

Try to sudo.

Then cd to the VM_PATH .

the git folder from udacity will be cloned and pre-reqs installed.


