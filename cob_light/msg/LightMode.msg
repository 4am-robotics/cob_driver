uint8 mode                         # mode to switch on
uint8 NONE =           0 	         # will turn everything off
uint8 STATIC =         1 	         # will change the LEDs to "color"
uint8 FLASH =          2 	         # will change the LEDs frequently with "frequency" from "color" to black
uint8 BREATH =         3           # will change the LEDs smoothly with "frequency" from "color" to black
uint8 BREATH_COLOR =   4           # will change the LEDs smoothly with "frequency" from "color" to black
						                       # and flips color in time
uint8 FADE_COLOR =     5           # will fade the colors in rainbow
uint8 SEQ =            6           # executes one sequence after another as definied in sequences array
uint8 CIRCLE_COLORS =  7           # circle through specific colors, if one color is set only one color will circle
                                   # if more than one color is set, that this colors will circle through

std_msgs/ColorRGBA     color       #the color which will be used
float32                frequency   # in Hz
float32                timeout 	   # in s, requested mode will be executed for max timout s.
						                       # default is 0 and meens no timeout.
int32                  pulses      # spezifies the amount of pulses which will be executed.
                                   # eg: mode = flash, pulses = 2. Meens the light will flash two times
int8                   priority    # priority [-20,20] default = 0. Modes with same or higher priorities will 
                                   # be executed.
std_msgs/ColorRGBA[]   colors      # array of colors for setting all the LEDs
cob_light/Sequence[]   sequences   # an array of sequence definitions, used only if mode is set to SEQ
