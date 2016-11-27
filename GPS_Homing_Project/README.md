#Arduino GPS Homing Device

This is an adaptation of a Adafruit project https://learn.adafruit.com/flora-gps-jacket/wear-it?view=all

The code has been changed so that given GPS coordinates a 12 LED Neopixel ring will light up with the LED that coresponds to the direction towards the end-point. The color of the LED's has been set up so that when you are headed directly towards the end-point the 0 index LED (12 o'clock) will be green, if the end-point is off to either the left or right the LED (2-5 and 7-11) that coresponds with the heading will range from yellow-green to yellow to red-orange, and if the heading is directly behind you the 6 index LED (6 o'clock) shows red. The intensity of the color has been set so the closer you get to the end-point the brighter the LED will show in its' respective color.

The end goal of this project will be to implement it into my car so that the hardware is hidden and the neopixel ring is all that shows.
