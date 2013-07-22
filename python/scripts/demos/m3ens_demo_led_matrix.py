#! /usr/bin/python

#Copyright  2008, Meka Robotics
#All rights reserved.
#http://mekabot.com

#Redistribution and use in source and binary forms, with or without
#modification, are permitted. 


#THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.

import m3.component_factory as m3f
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import time
from PIL import Image
from PIL import ImageDraw
import math

# ########################################
proxy = m3p.M3RtProxy()
proxy.start()
cnames=proxy.get_available_components('m3led_matrix_ec')
if len(cnames)!=1:
    print 'Invalid number of boards'
    exit()
print 'C',cnames
led = m3f.create_component(cnames[0])
print 'LED',led.name
proxy.publish_command(led) 
proxy.publish_param(led) 
proxy.subscribe_status(led)
proxy.make_operational_all()
led.enable_leds()
proxy.step()
# ########################################
print 'Direct mode [n]?'
if m3t.get_yes_no('n'):
    try:
        while True:
	    print 'Slew rate [',led.config['param']['slew_rate'],']?'
	    led.set_slew_rate(m3t.get_float(led.config['param']['slew_rate']))
            print 'R:'
            r=m3t.get_int()
            print 'G:'
            g=m3t.get_int()
            print 'B:'
            b=m3t.get_int()
            for rr in range(8):
                for cc in range(16):
                    led.set_rgb(rr,cc,[r,g,b])
            proxy.step()
            time.sleep(1.0)
    except (KeyboardInterrupt,EOFError):
        led.disable_leds()
        proxy.step()
        proxy.stop()
        exit()
# ########################################
print 'Animation mode [n]?'
if m3t.get_yes_no('n'):
    print 'Slew rate [',led.config['param']['slew_rate'],']?'
    led.set_slew_rate(m3t.get_float(led.config['param']['slew_rate']))
    led.load_animation('test')
    led.start_animation('test',cycle=True)
    try:
        while True:
            proxy.step()
            time.sleep(0.1)
    except (KeyboardInterrupt,EOFError):
	led.disable_leds()
	proxy.step()
	proxy.stop()  
	exit()   
 

# ########################################
class LineMouth:
    def __init__(self,led):
	self.led=led
	self.nc=self.led.num_col
	self.nr=self.led.num_row
	self.color_a=[0,0,255]
	self.color_b=[255,0,0]
	self.image={'neutral':self.led.get_image('neutral').load(),'smile':self.led.get_image('smile').load()}
	#print 'Slew rate [',led.config['param']['slew_rate'],']?'
	led.set_slew_rate(2000)
	self.cnt=0

    def step(self):
	self.cnt=(self.cnt+1)%200
	freq=0.1
	ss=0.5*(math.sin(time.time()*2*math.pi*freq)+1)
	r=int(ss*self.color_a[0]+(1-ss)*self.color_b[0])
	g=int(ss*self.color_a[1]+(1-ss)*self.color_b[1])
	b=int(ss*self.color_a[2]+(1-ss)*self.color_b[2])
	if self.cnt<100:
	    pix=self.image['neutral']
	else:
	    pix=self.image['smile']
	for rr in range(self.led.num_row):
	    for cc in range(self.led.num_col):
		x=[0,0,0]
		if pix[cc,rr][0]!=0: #map color to binary image
		    x[0]=r
		if pix[cc,rr][1]!=0:
		    x[1]=g
		if pix[cc,rr][2]!=0:
		    x[2]=b
		self.led.set_rgb(rr,cc,x)

print 'LineMouth mode [n]?'
if m3t.get_yes_no('n'):
    lm=LineMouth(led)
    try:
        while True:
	    lm.step()
            proxy.step()
            time.sleep(0.1)
    except (KeyboardInterrupt,EOFError):
        led.disable_leds()
	proxy.step()
	proxy.stop()  
	exit()
    
# ########################################
class ImageTester:
    def __init__(self,led):
		self.led=led
		self.nc=self.led.num_col
		self.nr=self.led.num_row
		self.color_a=[0,0,200]
		self.color_b=[200,0,0]
		images=self.led.get_available_images()
		self.image={}
		
		for x in images:
			name=x[x.rfind('/')+1:x.rfind('.')]
			self.image[name]=self.led.get_image(name).load()
			
		led.set_slew_rate(2000)
		self.cnt=0
		self.pix=None
		
    def step(self):
		freq=0.1
		ss=0.5*(math.sin(time.time()*2*math.pi*freq)+1)
		r=int(ss*self.color_a[0]+(1-ss)*self.color_b[0])
		g=int(ss*self.color_a[1]+(1-ss)*self.color_b[1])
		b=int(ss*self.color_a[2]+(1-ss)*self.color_b[2])
		
		if self.cnt==0:
			print 'Image?'
			k=self.image.keys()
			for idx in range(len(k)):
				print idx,' : ',k[idx]
			i=m3t.get_int()
			self.pix=self.image[k[i]]
			
		for rr in range(self.led.num_row):
			for cc in range(self.led.num_col):
				x=[0,0,0]
				if self.pix[cc,rr][0]!=0: #map color to binary image
					x[0]=r
				if self.pix[cc,rr][1]!=0:
					x[1]=g
				if self.pix[cc,rr][2]!=0:
					x[2]=b
				self.led.set_rgb(rr,cc,x)
			
		self.cnt=(self.cnt+1)%20
	
print 'Image Tester [y]?'
if m3t.get_yes_no('y'):
    lm=ImageTester(led)
    try:
        while True:
	    lm.step()
            proxy.step()
            time.sleep(0.1)
    except (KeyboardInterrupt,EOFError):
        led.disable_leds()
	proxy.step()
	proxy.stop()  
	exit()
	
class KnightRider:
    def __init__(self,led):
		self.led=led
		self.nc=self.led.num_col
		self.nr=self.led.num_row
		self.rgb_base=(255,0,0)
		self.rgb_ball=(100,0,255)
		self.rate=1
		self.ball=[self.nc/2,self.nr/2]
		self.ball_step=1.0
		print 'Slew rate [',led.config['param']['slew_rate'],']?'
		led.set_slew_rate(m3t.get_float(led.config['param']['slew_rate']))

    def step(self):
		bsz=3
		im = Image.new("RGB", (self.led.num_col,self.led.num_row), (0,0,0))
		draw = ImageDraw.Draw(im)
		draw.rectangle((self.ball[0]-1,self.ball[1]-2,self.ball[0]+1,self.ball[1]+1),fill=self.rgb_ball)#ellipse
		#draw.line((0,self.ball[1],self.nc-1,self.ball[1]),fill=self.rgb_base)
		self.ball[0]=self.ball[0]+self.ball_step
		if self.ball[0]>self.nc-1:
			self.ball[0]=self.nc-1
			self.ball_step=self.ball_step*-1
		if self.ball[0]<1:
			self.ball[0]=1
			self.ball_step=self.ball_step*-1
		del draw 
		return im

    
print 'Knight Rider mode [n]?'
if m3t.get_yes_no('n'):
    kn=KnightRider(led)
    try:
        while True:
	    led.set_image(kn.step())
            proxy.step()
            time.sleep(0.1)
    except (KeyboardInterrupt,EOFError):
        led.disable_leds()
	proxy.step()
	proxy.stop()  
	exit()
    
# ########################################
