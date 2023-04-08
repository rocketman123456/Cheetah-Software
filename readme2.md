
## build

	mkdir build
	
	cd build
	
	cmake -DMINI_CHEETAH_BUILD=TRUE ..
	
	./../scripts/make_types.sh
	
	make -j4



## change libstdc++.so version if needed

libstdc++.so.6.25 -->  libstdc++.so.6.21

### gcc version change

* brefore

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 80

sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 80

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 70

sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 70


* after make

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 50

sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 50


## other change

* set the IP of the script && send

../scripts/send_to_mini_cheetah.sh user/MIT_Controller/mit_ctrl
