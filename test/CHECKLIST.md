# Checklist for testing the Vine System

- [ ] Front Inner LEDs do not work
- [X] Add scripts for lighting control (these also act as documentation)
- [X] Add script to find connected Pi Cams and provide index
- [ ] Docker Compose doesn't handle the xserver well, we need to manually do `xhost +local:docker` each time before starting, fix this
