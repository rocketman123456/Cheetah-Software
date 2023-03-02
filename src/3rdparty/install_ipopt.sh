# copy from quadruped_ctrl
# Setup and build for IPOPT
cd ipopt
sudo apt-get install -y gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev
if [ -d "./coinbrew" ] 
then
	sudo rm -r ./coinbrew
fi
git clone https://www.github.com/coin-or/coinbrew
cd coinbrew
chmod u+x coinbrew
./coinbrew fetch Ipopt --latest-release --no-prompt
cd ..
if [ -d "./coinhsl" ] 
then
    echo "HSL found." 
	cp -r ./coinhsl ./coinbrew/ThirdParty/HSL/coinhsl
else
    echo "Warning: HSL not found."
fi
cd coinbrew
sudo ./coinbrew build Ipopt --tests none --prefix=/usr/local --no-prompt --parallel-jobs=8
cd ../..
