#!/usr/bin/python

import datetime
import time

while True :
  f = open("/sys/class/range_sensor/value",'r')
  d = f.read()
  f.close()
	
  i = datetime.datetime.now()
  print "[%s] %.1f cm (%.1f in)" % (i.isoformat(), float(d)/58, float(d) / 148) 

  time.sleep(0.2)
