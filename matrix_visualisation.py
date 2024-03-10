import re
import time
import argparse
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.core.virtual import viewport
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, TINY_FONT, SINCLAIR_FONT, LCD_FONT

#print in console
def output_in_console(output_data):
	for i in range(8):
		print(output_data[i])
		
def output_in_max7219_matrix(n, block_orientation, rotate, inreverse, output_data):
	# create matrix device
	serial = spi(port=0, device=0, gpio=noop())
	device = max7219(serial,cascaded=n or 1,block_orientation=block_orientation,rotate=rotate or 0,blocks_arranged_in_reverse_order=inreverse)
	
	with canvas(device) as draw:
		for i in range(len(output_data)):
			for j in range(len(output_data[i])):
				if (output_data[i][j] == 2):
					draw.point((i, j), fill = "red")
