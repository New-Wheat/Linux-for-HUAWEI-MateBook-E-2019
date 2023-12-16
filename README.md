# Linux for HUAWEI MateBook E 2019



## Status

Support features (requires proper firmware files placed in `/lib/firmware`):
    
- volume key
- power key
- touchscreen
- stylus
- wifi
- bluetooth
- usb
- keyboard
- touchpad
- ufs
- audio (EXTREMELY unstable)
- gpu
- cellular network

## Todo


- [ ] Camera
    - front camera: gc5025 (no sensor driver found)
    - rear camera: s5k3l6 (wip)
- [ ] Battery monitoring
    - needs special EC driver (attached to `&i2c6` at 0x76)
- [ ] Backlight
    - needs special EC driver (attached to `&i2c6` at 0x76)

## Source Code

Linux kernel source code: https://gitlab.com/sdm845-mainline/linux

s5k3l6 camera sensor driver: https://source.puri.sm/Librem5/linux/-/blob/pureos/latest/drivers/media/i2c/s5k3l6xx.c

## Testing Platform

Debian 12
