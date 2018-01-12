
#sudo apt-get install autoconf automake libtool

./autogen.sh 

cp Makefile.in_dong Makefile.in

./configure --enable-bitmain-D1 --without-curses --host=arm-xilinx-linux-gnueabi --build=x86_64-pc-linux-gnu
