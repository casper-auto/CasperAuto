#! /bin/bash

# update
sudo apt-get update

# gfortran dependency
sudo apt-get install -y gfortran

# get unzip
sudo apt-get install -y unzip

# Ipopt: get, install, unzip
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.11.zip && unzip Ipopt-3.12.11.zip && rm Ipopt-3.12.11.zip
./install_ipopt.sh ./Ipopt-3.12.11

# CppAD
sudo apt-get install -y cppad

# Gnuplot
sudo apt-get install -y gnuplot

#Eigen3
sudo apt install -y libeigen3-dev

# python3 and matplotlib
sudo apt install -y python3-matplotlib python3-dev
