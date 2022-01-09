# H747-Camera

The hardware setup is done using CubeMX to create the project with AzureRTOS
threadX enabled on both cores.  The DCMI, LTDC and DSI are also configured from
CubeMX.

A camera module needs to be attached to the main board.

- M4 will :
  0. Start DMA on DCMI in continuous mode
  1. toggle the orange LED twice per second using 1 tx thread
  2. restart DMA when the frame complete event is received from DCMI,
     write some things on the UART once per second using 1 tx thread
  3. read data from I2C4 and put the retrieved data in shared memory for M7
     using 1 tx thread

- M7 will :
  1. toggle blue LED once per second; retrieve touch screen data from the shared
     memory wrt attached touchscreen and post PEN events for guix
  2. start guix using 1 tx thread - a few GUIX items are displayed

GUIX was added as a git submodule

The display is generated from GUIX Studio and the large font is also generated
by GS from the Cascadia Code font available here :
https://github.com/microsoft/cascadia-code/releases

