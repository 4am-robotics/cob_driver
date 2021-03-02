uint8 NONE =           0           # will turn everything off
uint8 STATIC =         1           # will change the LEDs to "color"
uint8 FLASH =          2           # will change the LEDs frequently with "frequency" from "color" to black
uint8 BREATH =         3           # will change the LEDs smoothly with "frequency" from "color" to black
uint8 BREATH_COLOR =   4           # will change the LEDs smoothly with "frequency" from "color" to black
                                   # and flips color in time
uint8 FADE_COLOR =     5           # will fade the colors in rainbow
uint8 SEQ =            6           # executes one sequence after another as defined in sequences array
uint8 CIRCLE_COLORS =  7           # circle through specific colors, if one color is set only one color will circle
                                   # if more than one color is set, that this colors will circle through
uint8 SWEEP =          8           # circle color from front to back on both sides
uint8 DIST_APPROX =    9           # color led circle from green to red depended on the measured laserscan dists
uint8 GLOW =          10           # glow the color
uint8 XMAS =          11           # red white christmas mode
uint8 KIT =           12           # Knight Rider mode
uint8 TURN_LEFT =     13           # Turn Indicator left
uint8 TURN_RIGHT =    14           # Turn Indicator right
