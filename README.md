# TI_RTOS_beaglebone_black

Assuming that you have a previous knowledge about beaglebone black or another embedded system project and programming workflow, this repository should be able to introduce you to RTOS programming for the AM335x processor at the beaglebone board with a few examples of the mostly used features as GPIO and I2C communication.

## My example Projects
These are my three example projects developed during RTOS discipline at college:

### Ex1

Simulates a four tasks system that can choose between two operation modes. The first have two running with RM scheduling. The other one with a hardware interruption (by a push button) put the four tasks to execute simultaneously. The execution time is simulated by a busy-waiting 

### Ex2

Implements a system that uses I2C communication to analyse information provided by a [MPU-6050](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) and sends these info via serial on a JSON format. The JSON format cant be showed by a serial plotter implemented on python

### Ex3

This exammple implements a RTOS with two tasks, the first wich does the initia configuration and calibration of MPU-6050 and another wich controls a routine that do a specified car route with a robot car provided for this test. The route is selected by a push button interrupt.

## Getting Started


These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. 

### Prerequisites

To build and run the examples and projects on a Linux system, you must have installed on your machine the TI-RTOS SDK and the Code Composer Studio IDE for a proper development and a serial monitor for Debugging

### Installing

To be able to build and run these examples on your machine, you have a few environment install and configuration steps as described below:
* Install *minicom* or another serial monitor of your preference
`sudo apt install minicom`
* Go to AM335x's processor SDK [webpage](http://www.ti.com/tool/PROCESSOR-SDK-AM335X) an choose  from *'PROCESSOR-SDK-RTOS-AM335X'* [*Get Software*](http://software-dl.ti.com/processor-sdk-rtos/esd/AM335X/latest/index_FDS.html)
* Install TI-RTOS SDK 
	* Before install the SDK and CCS, you need to install a few packages on your machine for ensure compatibility and that will work
	```
	$ sudo apt install libc6-i386
	$ sudo apt install libusb-0.1-4
	$ sudo apt install libgconf-2-4
	$ sudo apt install build-essential
	```
	* At this project development the newest version of SDK was the [ti-processor-sdk-rtos-am335x-evm-05.03.00.07](http://software-dl.ti.com/processor-sdk-rtos/esd/AM335X/latest/exports/ti-processor-sdk-rtos-am335x-evm-05.03.00.07-Linux-x86-Install.bin) but you can choose the the latest version for your project
	* After the instalation process finish go to the processor's directory and configure the environment variables:
	```
	$ cd ~/ti/processor_sdk_rtos_am335x_5_03_00_07/
	$ ./setupenv.sh
	$ make
	 ``` 
* Install Code Composer Studio
	* At this project development the newest version of CCS was the [CCS8.3.0.00009](http://software-dl.ti.com/ccs/esd/CCSv8/CCS_8_3_0/exports/CCS8.3.0.00009_web_linux-x64.tar.gz) but you can choose the the latest version for your project
	* During the instalation steps 
		* At Processor support - choose 'Sitara AMx Processors'
		* Then the setup shows unsupported boards
		* At Select Debug Probes - choose none (*unless you have the ways to use this feature*)
	* After the installation finishes, go to the PDK's directory and configure the environment variables for the creation of the example projects:
	```
	$ cd ti/pdk_am335x_1_0_14/packages/
	$ source ./pdksetupenv.sh
	$ ./pdkProjectCreate.sh
	```
	* Now you should be able to import the example projects on CCS.


* *(Optional but recommended)* After running the downloaded file, you can override some configuration files that i have already changed for configure the others GPIO modules and enable I2C communication as described on the [*'configuration_files'*](https://github.com/ferreiracaf/TI-RTOS_beaglebone_black/tree/master/configuration_files) directory 