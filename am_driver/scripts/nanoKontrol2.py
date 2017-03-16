#! /usr/bin/env python
import os, sys
import pygame.midi
import pygame.event
import array
import ctypes
import math
import time
import ConfigParser

# ROS
import rospy
from sensor_msgs.msg import *

# Controls:
# 0x00 - 0x07: sliders
# 0x10 - 0x17: knobs
# 0x20 - 0x27: S buttons
# 0x30 - 0x37: M buttons
# 0x40 - 0x47: R buttons

# Buttons
# Cycle = 46
# Marker Set = 60
# Marker < = 61
# Marker > = 62
# << = 43
# >> = 44
# |_| = 42
# |> = 41
# O = 45
# Track < = 58
# Track > = 59

MI_CYCLE = 0
MI_MK_SET = 1
MI_MK_FWD = 2
MI_MK_BWD = 3
MI_PB_BWD = 4
MI_PB_FWD = 5
MI_PB_STOP = 6
MI_PB_PLAY = 7
MI_PB_REC = 8
MI_TRACK_FWD = 9
MI_TRACK_BWD = 10

MI_EVENT_TYPE_SLIDER = pygame.USEREVENT + 1
MI_EVENT_TYPE_KNOB = pygame.USEREVENT + 2
MI_EVENT_TYPE_SMR = pygame.USEREVENT + 3
MI_EVENT_TYPE_KEY = pygame.USEREVENT + 4

MI_S_START = 0x20
MI_M_START = 0x30
MI_R_START = 0x40

theNanoKontrol2Instance = None


class nanoKontrol2:
	def __init__(self):
		global theNanoKontrol2Instance
		theNanoKontrol2Instance = self
		
		pygame.midi.init()
		# attempt to autodetect nanokontrol
		(in_device_id, out_device_id) = self.findNanoKontrol()

		if in_device_id == None or out_device_id == None:
			self.midi_in = None
			self.midi_out = None
			pygame.midi.quit()
			return

		self.midi_in = self.midi_in = pygame.midi.Input( in_device_id )
		print "[nanoKontrol2] using input  id: %s" % in_device_id

		self.midi_out = self.midi_out = pygame.midi.Output(out_device_id, 0)
		print "[nanoKontrol2] using output id: %s" % out_device_id
		
		self.keyTranslate = {	46: MI_CYCLE, 
								60: MI_MK_SET, 
								62: MI_MK_FWD,
								61: MI_MK_BWD,
								43: MI_PB_BWD,
								44: MI_PB_FWD,
								42: MI_PB_STOP,
								41: MI_PB_PLAY,
								45: MI_PB_REC,
								58: MI_TRACK_FWD,
								59: MI_TRACK_BWD
								 }
		self.keyTranslateInv = {MI_CYCLE:46, 
								MI_MK_SET:60, 
								MI_MK_FWD:62,
								MI_MK_BWD:61,
								MI_PB_BWD:43,
								MI_PB_FWD:44,
								MI_PB_STOP:42,
								MI_PB_PLAY:41,
								MI_PB_REC:45,
								MI_TRACK_FWD:58,
								MI_TRACK_BWD:59
								 }

		self.light(MI_CYCLE, False)
		self.light(MI_MK_SET, False)
		self.light(MI_MK_BWD, False)
		self.light(MI_MK_FWD, False)
		self.light(MI_PB_BWD, False)
		self.light(MI_PB_FWD, False)
		self.light(MI_PB_STOP, False)
		self.light(MI_PB_PLAY, False)
		self.light(MI_PB_REC, False)
		self.light(MI_TRACK_BWD, False)
		self.light(MI_TRACK_FWD, False)
		
		for i in range(0, 8):
			self.light(MI_S_START + i, False)
			self.light(MI_M_START + i, False)
			self.light(MI_R_START + i, False)
        
	# both display all attached midi devices, and look for ones matching nanoKONTROL2
	def findNanoKontrol(self):
		print "[nanoKontrol2] ID: Device Info"
		print "[nanoKontrol2] ---------------"
		in_id = None
		out_id = None
		for i in range( pygame.midi.get_count() ):
			r = pygame.midi.get_device_info(i)
			(interf, name, input, output, opened) = r

			in_out = ""
			if input:
				in_out = "(input)"
			if output:
				in_out = "(output)"

			if "nanoKONTROL2" in name and input:
				in_id = i
			elif "nanoKONTROL2" in name and output:
				out_id = i
			print ("[nanoKontrol2] %2i: interface :%s:, name :%s:, opened :%s:  %s" %(i, interf, name, opened, in_out))
		return (in_id, out_id)

	# turn a LED on or off
	def light(self, key, on):
		if on:
			out = 127
		else:
			out = 0
		
		try:
			btn = self.keyTranslateInv[key]
		except:
			btn = key
			
		self.midi_out.write_short(176, btn, out)

	def getEvents(self):
		
		if self.midi_in == None:
			return []
		
		events = []
		
		if self.midi_in.poll():
			midi_events = self.midi_in.read(100)
			midi_evs = pygame.midi.midis2events(midi_events, self.midi_in.device_id)

			for me in midi_evs:
				#print "EV:", me.data1, me.data2

				# midi sliders
				if me.data1 >= 0x00 and me.data1 <= 0x07:
					ev = pygame.event.Event(MI_EVENT_TYPE_SLIDER, {'id':me.data1, 'value':me.data2})
					events.append(ev)

				# midi knobs
				if me.data1 >= 0x10 and me.data1 <= 0x17:
					ev = pygame.event.Event(MI_EVENT_TYPE_KNOB, {'id':me.data1 - 0x10, 'value':me.data2})
					events.append(ev)

				# button RMS
				if (me.data1 >= 0x20 and me.data1 <= 0x27) or (me.data1 >= 0x30 and me.data1 <= 0x37) or (me.data1 >= 0x40 and me.data1 <= 0x47):
					ev = pygame.event.Event(MI_EVENT_TYPE_SMR, {'id':me.data1 - 0x20, 'keydown':me.data2/127})
					events.append(ev)

				if me.data1 in self.keyTranslate:
					ev = pygame.event.Event(MI_EVENT_TYPE_KEY, {'id':self.keyTranslate[me.data1], 'keydown':me.data2/127})
					events.append(ev)

		
		return events

	def close(self):
		print "[nanoKontrol2] Close MIDI"
		self.midi_in = None
		self.midi_out = None
		pygame.midi.quit()


class RosNano2:
	def __init__(self):
		# init the controller
		self.midi = nanoKontrol2()

		self.pub = rospy.Publisher('/nano2', Joy)

		self.update_rate = 10   # Hz
		
		self.msg = Joy()
		self.msg.axes = [ 0 ] * 18
		self.msg.buttons = [ 0 ] * (25+24)

	def convertEventToJoy(self, e):
		if e.type == MI_EVENT_TYPE_SLIDER:
			#print "Slider:", e.id, e.value
			self.msg.axes[e.id] = float(e.value)/127.0
		elif e.type == MI_EVENT_TYPE_KNOB:
			#print "Knob:", e.id, e.value
			self.msg.axes[8+e.id] = float(e.value)/127.0
		elif e.type == MI_EVENT_TYPE_KEY:
			#print "Button:", e.id, e.keydown
			self.msg.buttons[e.id] = e.keydown
		elif e.type == MI_EVENT_TYPE_SMR:
			# e.id is 0-8 for S, 16-24 for M, and 32-40 for R
			# Remap these to fit after the current buttons
			# print "SMR:", e.id, e.keydown, e.type
			modId = e.id
			if modId >= 32:
				modId -= 16
			elif modId >=16:
				modId -= 8	 
				
			self.msg.buttons[25+modId] = e.keydown
		
	def update(self):
		# Read the controller and publish...
		self.msg.header.stamp = rospy.Time.now()
		events = self.midi.getEvents()
		for e in events:
			self.convertEventToJoy(e)
		
		if len(events) > 0:
			#print self.msg
			self.pub.publish(self.msg)
		
	def run(self):
		try:
			r = rospy.Rate(self.update_rate) # Hz
			while not rospy.is_shutdown():
				self.update()
				r.sleep()
		except rospy.exceptions.ROSInterruptException:
			pass
		finally:
			self.midi.close()
			self.midi = None


########################################################################
## Unit test
########################################################################
if __name__ == '__main__':

	going = True
	
	########################################################################
	## Ctrl-C signal for UnitTest
	########################################################################
	import signal

	def signal_handler(signal, frame):
		global going
		print 'You pressed Ctrl+C!'
		going = False

	signal.signal(signal.SIGINT, signal_handler)
	
	rospy.init_node('nano_kontrol')
	srv = RosNano2()
	srv.run()
