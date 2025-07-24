# Linux for HUAWEI MateBook E 2019

[Devicetree](https://github.com/New-Wheat/Linux-for-HUAWEI-MateBook-E-2019/blob/main/sdm850-huawei-matebook-e-2019.dts) and extra resources to run Linux on HUAWEI MateBook E 2019

## Status

Supported features (some require proper firmware files placed in `/lib/firmware`):

- Volume Key
- Power Key
- Touchscreen
- Stylus
- WiFi
- Bluetooth
- GPU
- USB
- Keyboard
- Touchpad
- UFS
- SD Card
- Audio _(**UNSTABLE**, right internal mic and headphone mic not enabled)_
- Mobile Network
- Camera Indicator LED  
- Rear Camera _(VCM driver for the actuator is [here](https://github.com/New-Wheat/Linux-for-HUAWEI-MateBook-E-2019/blob/main/drivers/media/i2c/cn3927e.c))_
- _**The following features require [huawei-planck-ec](https://github.com/New-Wheat/Linux-for-HUAWEI-MateBook-E-2019/blob/main/drivers/platform/arm64/huawei-planck-ec.c) driver**_
    - Battery Monitoring
    - Lid
    - Backlight
    - USCI 
- _**The following features require libssc and hexagonrpcd (See [here](https://gitlab.com/postmarketOS/pmaports/-/merge_requests/4050))**_
    - Accelerometer
    - Ambient Light Sensor


## Todo

- [ ] External display _(video output via USB-C currently works very badly)_
- [ ] Camera
    - front camera: gc5025 (only downstream driver found)

## Source Code

Linux kernel source code: https://gitlab.com/sdm845-mainline/linux

s5k3l6 camera sensor driver: https://source.puri.sm/sebastian.krzyszkowiak/linux-next/-/blob/dos-6.9/drivers/media/i2c/s5k3l6xx.c

## Testing Platform

Debian 12 (Linux kernel 6.14-rc5)
