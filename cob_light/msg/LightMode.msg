uint8 mode                         # mode to switch on

float32                frequency   # in Hz
float32                timeout     # in s, requested mode will be executed for max timeout s.
                                   # default is 0 and means no timeout.
int32                  pulses      # specifies the amount of pulses which will be executed.
                                   # eg: mode = flash, pulses = 2. Means the light will flash two times
int8                   priority    # priority [-20,20] default = 0. Modes with same or higher priorities will
                                   # be executed.
std_msgs/ColorRGBA[]   colors      # array of colors.
                                   # size > 1: each color represent one led (size should match the amount of leds)
                                   # size = 1: one color for all leds or color for specific mode
                                   # size = 0: undefined
cob_light/Sequence[]   sequences   # an array of sequence definitions, used only if mode is set to SEQ
