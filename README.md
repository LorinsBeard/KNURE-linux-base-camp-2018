# Smart Door project

Simple door lock with NFC card reader.
All drivers and logic are linux kernel modules.

## Components and connection

### Main components

* [OrangePi](http://www.orangepi.org/orangepione/) - target board
* [MFRC522](http://www.kosmodrom.com.ua/el.php?name=MFRC-522-RFID-KIT) - NFC cards reader
* [L298N](http://www.kosmodrom.com.ua/el.php?name=L298N-MOTOR-DRIVER) - motor driver
* [DC-DC](http://www.kosmodrom.com.ua/el.php?name=DC-DC-MINI-3A) - power module for target board
* [Door Lock])https://topzamok.com.ua/tovar/zamki/zamok-zadvizhka-nevidimka-gardian-emz-1-02) - locking mechanism

### Backplane connection

![Backplane](https://raw.githubusercontent.com/LorinsBeard/KNURE-linux-base-camp-2018/Project_SmartDoor/Documents/backplane_pinout.png)


## Getting started

### Prerequisites
Before compile module some variables should be changed:

In envsetup.sh file You should change:

TARGET_ROOT - location of module sourses
BUILD_KERNEL - location of kernel sourses


In build_on_x86.sh You should change:

SSH_NAME - ssh name of target board (if not used keep empty)
OR
BOARD_USER - target board user
BOARD_IP - IP address of target board


### Building
Enviroment setup:
```
cd SmartLockControl/
source envsetup.sh
```
Building sources:
```
./build_on_x86.sh --clean --module
```
Building DTB file:
```
./build_on_x86.sh --dtb
```
Deploying module to the target board:
```
./build_on_x86.sh --deploy
```

### Built with
arm-linux-gnueabihf - version 7.3.0-3ubuntu2.1 
linux kernel - version 4.17

## Authors

*   [**Maksim Holikov**](https://github.com/MaksimGolikov)
*   [**Oleksii Klochko**](https://github.com/LorinsBeard)
*   [**Yevgen Kovalyov**](https://github.com/yekovalyov)

Linux kernel base camp by KNURE for GlobalLogic Kharkiv, 2018
