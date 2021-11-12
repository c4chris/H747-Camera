# H747-Camera

The hardware setup is done using CubeMX to create the project with AzureRTOS
threadX enabled on both cores and USBX host mode for HID enabled on M7.  The
LTDC and HDMI are also configured from CubeMX.

- M4 will :
  1. toggle the orange LED twice per second using 1 tx thread
  2. write things on the UART once per second using 1 tx thread
  3. read data from I2C1 and put the retrieved data in shared memory for M7
     using 1 tx thread
  4. read data from I2C4, same as above

- M7 will :
  1. initialize usbx (using several tx threads) in host-only mode and handling
     mouse or touchscreen HID
  2. toggle blue LED once per second; retrieve touch screen data from the USB
     attached device and post PEN events for guix
  3. start guix using 1 tx thread and setup a timer to periodically update a
     displayed value obtained from the shared memory (I2C data sent by M4) --
     currently the I2C4 device is not connected to the board and the value of a
     counter is displayed instead

GUIX was added as a git submodule

The display is generated from GUIX Studio and the large font is also generated
by GS from the Cascadia Code font available here :
https://github.com/microsoft/cascadia-code/releases

