cd ../host_driver/esp32/
make -j8
sudo insmod esp32.ko
sudo mknod /dev/esps0 c 221 0
sudo chmod 666 /dev/esps0


