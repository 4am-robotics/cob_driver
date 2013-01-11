uint8 mode
uint8 NONE = 0 # this mode is used if the actual mode sould be used and only the color or frequency should be adapted
uint8 STATIC = 1 # will change the LEDs to "color"
uint8 FLASH = 2 # will change the LEDs frequently with "frequency" from "color" black
uint8 BREATH = 3 # will change the LEDs smoothly with "frequency" from "color" to black
uint8 BREATH_COLOR = 4 #will change the LEDs smoothly with "frequency" from "color" to black and flips color in time
uint8 FADE_COLOR = 5 #will fade the colors in rainbow

std_msgs/ColorRGBA color #the color which will be used
float32 frequency # in Hz
int32 timeout # in ms
int32 pulses # number of pulses
int8 priority # priority [-20,20] default = 0