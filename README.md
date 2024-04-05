# Linux for HUAWEI MateBook E 2019

[Devicetree](https://gitlab.com/New-Wheat/linux-for-huawei-matebook-e-2019/-/blob/main/sdm850-huawei-matebook-e-2019.dts) and other information to run Linux on HUAWEI MateBook E 2019

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
- Battery Monitoring    (requires huawei-planck-ec.c driver)
- Lid                   (requires huawei-planck-ec.c driver)
- Backlight             (requires huawei-planck-ec.c driver)
- Audio                 (EXTREMELY unstable, `status = "disabled"` by default)
- Mobile Network
- Camera Indicator LED

## Todo


- [ ] Camera
    - front camera: gc5025 (no sensor driver found)
    - rear camera: s5k3l6 (wip)

## Source Code

Linux kernel source code: https://gitlab.com/sdm845-mainline/linux

s5k3l6 camera sensor driver: https://source.puri.sm/Librem5/linux/-/blob/pureos/latest/drivers/media/i2c/s5k3l6xx.c

## Testing Platform

Debian 12
