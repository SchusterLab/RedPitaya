rsync -avz --exclude='.git/' --delete -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" --progress /cygdrive/c/Users/slab/Documents/GitHub/RedPitaya fpga@192.168.14.20:/home/fpga

ssh fpga@192.168.14.20 '/home/fpga/compile_local.sh'

rsync -avz --exclude='.git/' --delete -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" --progress fpga@192.168.14.20:/home/fpga/RedPitaya/fpga/out /cygdrive/c/Users/slab/Documents/GitHub/RedPitaya/fpga
