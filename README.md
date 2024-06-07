# Linux for HUAWEI MateBook E 2019

[Devicetree](https://gitlab.com/New-Wheat/linux-for-huawei-matebook-e-2019/-/blob/main/sdm850-huawei-matebook-e-2019.dts) and extra resources to run Linux on HUAWEI MateBook E 2019

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
- Audio _(**UNSTABLE**, only speakers and headphones are supported now)_
- Mobile Network
- Camera Indicator LED  
- _**The following features requires [huawei-planck-ec](https://gitlab.com/New-Wheat/linux-for-huawei-matebook-e-2019/-/blob/main/drivers/power/supply/huawei-planck-ec.c) driver**_
    - Battery Monitoring
    - Lid
    - Backlight  
- _**The following features requires libssc and hexagonrpcd (See [here](https://gitlab.com/postmarketOS/pmaports/-/merge_requests/4050))**_
    - Accelerometer
    - Ambient Light Sensor


## Todo


- [ ] Camera
    - front camera: gc5025 (no sensor driver found)
    - rear camera: s5k3l6 (wip)

## Source Code

Linux kernel source code: https://gitlab.com/sdm845-mainline/linux

s5k3l6 camera sensor driver: https://source.puri.sm/Librem5/linux/-/blob/pureos/latest/drivers/media/i2c/s5k3l6xx.c

## Testing Platform

Debian 12 (Linux kernel 6.9)
