import argparse

class Arguments():
	def __init__(self):
		parser = argparse.ArgumentParser()
		parser.add_argument('-x', action='store_true', dest = 'xbox',
							default = False,
							help='Turn on xbox listener',
							)
		parser.add_argument('-s', action='store_true', dest = 'unix',
							default = False,
							help='Turn on LED socket',
							)
		parser.add_argument('-b', action='store_true', dest = 'bt',
							default = False,
							help='Turn on bluetooth listener',
							)
		
		self.bt_flag = parser.parse_args().bt
		self.unix_flag = parser.parse_args().unix
		self.xbox_flag = parser.parse_args().xbox
