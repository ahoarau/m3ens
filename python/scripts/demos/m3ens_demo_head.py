#! /usr/bin/python
# -*- coding: utf-8 -*-

#Copyright  2012, Meka Robotics
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

import time
import m3.rt_proxy as m3p
import m3.toolbox as m3t
import m3.toolbox_beh as m3b
import m3.toolbox_head_s2 as m3h
import m3.component_factory as m3f
import m3.led_matrix_ec as m3l
import m3.head_s2csp_ctrl as m3csp
import m3.unit_conversion as m3u
#import m3.toolbox_ros as m3tr
import m3.pwr
import math
import random

# ######################################################	
class LookBehaviors:
	def __init__(self,bot,csp,beh,use_fd):
		self.joints=range(7)
		self.beh=beh
		self.bot=bot 
		self.csp=csp
		self.use_fd=use_fd
		self.csp.enable()
		self.t_rand=time.time()
		self.t_slew=[m3t.M3Slew(),m3t.M3Slew(),m3t.M3Slew()]
		self.tbox=m3h.M3HeadToolboxS2ENS(self.bot.get_chain_component_name('head'),bot)
		self.target_rand=[0,0]
		if use_fd:
			self.fd=m3tr.M3FaceDetectThread('right',verbose=False)
			self.fd.start()
			self.fd_last=time.time()
			self.fd_target=None
			self.fd_rects=None
	   
	
	def stop(self):
		self.csp.set_target_csp_frame(m3h.spherical_to_cartesian(0,0))
		if self.use_fd:
			self.fd.stop()
	
	def set_eye_slewrate_proportion(self,val):
		self.csp.set_slew_rate_proportion(4,val)
		self.csp.set_slew_rate_proportion(5,val)
		self.csp.set_slew_rate_proportion(6,val)
	
	def zero(self):
		target=m3h.spherical_to_cartesian(0,0)
		self.csp.set_target_csp_frame(target)
		self.set_eye_slewrate_proportion(0.06)
		#print 'Zero'
		return m3b.res_continue
	
	def roll_backforth(self):
		amp=10.0
		freq=0.2 #Hz
		des=amp*math.sin(time.time()*math.pi*2*freq)
		self.csp.set_theta_j2_deg(des)
	
	def roll_zero(self):
		self.csp.set_theta_j2_deg(0)
	
	def random(self):
		if time.time()-self.t_rand>6.0:
			self.t_rand=time.time()
			self.target_rand=[-10.0+(2*random.random()-1)*30.0,(2*random.random()-1)*70.0]
		self.csp.set_target_csp_frame(m3h.spherical_to_cartesian(self.target_rand[0],self.target_rand[1]))
		self.set_eye_slewrate_proportion(0.10)
		self.beh.restore_priority('eyelids','sleepy')
		return m3b.res_continue
	
	def leftright(self):
		self.set_eye_slewrate_proportion(0.5)
		self.csp.enable()
		freq=0.3
		des=25.0*math.sin(freq*2*math.pi*time.time())
		target=m3h.spherical_to_cartesian(0,des)
		self.csp.set_target_csp_frame(target)
		self.beh.restore_priority('eyelids','sleepy')
		return m3b.res_continue

	def face_detected(self):
		#print 'face_detected?'
		#if x>cx, increment x pos, else decr x pos, etc...
		self.fd_rects,dt=self.fd.get_detection()
		if self.fd_rects!=None and dt-self.fd_last>2.0:
			xi=[self.fd_rects[0].x,self.fd_rects[0].y]
			#xi=[457.698, 297.003]
			self.fd_target=self.tbox.image_2_world('right',xi,r=1.0)
				 #Target is in the head-base frame
			#print '-----------'
			#print 'Pixel',xi
			print 'FaceDetectTarget',self.fd_target
			#self.fd_target=[1.125,.5,.260]
			#print 'VirtualTarget',self.fd_target
			#self.fd_target-self.bot.eye_2_world(eye,xe)
			#print 'Xw',self.fd_target
			self.fd_last=dt
			return True
		return False
	
	def facetrack(self):
		if self.fd_target is not None:
			t=[self.t_slew[0].step(self.fd_target[0],0.05),
			   self.t_slew[1].step(self.fd_target[1],0.05),
			   self.t_slew[2].step(self.fd_target[2],0.05)]
			self.set_eye_slewrate_proportion(0.08)
			self.csp.set_target_head_base_frame(self.tbox.world_2_head_base(t))
			print 'Facetrack: ',t#World',t,'HeadBase',self.tbox.world_2_head_base(t)
			return m3b.res_continue
		return m3b.res_finished
	

	
class EyelidBehaviors:
	def __init__(self,bot,beh):
		self.joints=[7]
		self.beh=beh
		self.blink_close=True
		self.bot=bot
		self.tbox=m3h.M3HeadToolboxS2ENS(self.bot.get_chain_component_name('head'),self.bot)
		self.q_max=self.tbox.eyelid_q_max
		self.q_min=self.tbox.eyelid_q_min
		
	def stop(self):
		self.bot.set_theta_deg('head',[35.0],self.joints)

	def limit(self,des):
		q=self.bot.get_theta_deg('head')
		l=self.tbox.step_eyelids_limit(q[4],des)
		return l

	def neutral(self):
		self.bot.set_mode_theta_mj('head',self.joints)
		self.bot.set_theta_deg('head',[55.0],self.joints)
		self.bot.set_thetadot_deg('head',[150],self.joints)
		self.beh.restore_priority('look','zero')
		return m3b.res_continue

	def sleepy(self):
		self.bot.set_mode_theta_mj('head',self.joints)
		self.bot.set_theta_deg('head',[self.limit(self.q_min)],self.joints)
		self.bot.set_thetadot_deg('head',[150.0],self.joints)
		self.beh.set_priority('look','zero',5) #ensure eyes fwd when lids closed
		return m3b.res_continue

	def blink(self): #does one blink
		self.beh.restore_priority('look','zero')
		if self.blink_close:
			des=self.limit(self.q_min)
		else:
			des=55.0
		q=self.bot.get_theta_deg('head')[self.joints[0]]
		err=abs(des-q)
		if self.blink_close and err<10.0:
			self.blink_close=False
		elif not self.blink_close and err<10.0:
			self.blink_close=True
			return m3b.res_finished
		self.bot.set_mode_theta_mj('head',self.joints)
		self.bot.set_theta_deg('head',[des],self.joints)
		self.bot.set_thetadot_deg('head',[300.0],self.joints)
		return m3b.res_continue


class LedBehaviors:
	def __init__(self,led,bot):
		self.led=led
		self.bot=bot
		self.image={'neutral':self.led.get_image('neutral').load(),'smile':self.led.get_image('smile').load()}
		self.led.set_slew_rate(450)
		self.colors={'pink':		[235,20,50],
					 'red':			[235,0,0],
					 'pinkpurp':	[150,20,200],
					 'purple':		[80,0,150],
					 'off':			[0,0,0],
					 'blue':		[20,0,250],
					 'green':		[20,250,20]}
	
	
	def binary_map_color_to_image(self,pix,rgb):
		for rr in range(self.led.num_row):
			for cc in range(self.led.num_col):
				x=[0,0,0]
				if pix[cc,rr][0]!=0: #map color to binary image
					x[0]=rgb[0]
				if pix[cc,rr][1]!=0:
					x[1]=rgb[1]
				if pix[cc,rr][2]!=0:
					x[2]=rgb[2]
				self.led.set_rgb(rr,cc,x)
	
	def color_cycle(self,color_a,color_b):
		freq=0.1
		ss=0.5*(math.sin(time.time()*2*math.pi*freq)+1)
		r=int(ss*color_a[0]+(1-ss)*color_b[0])
		g=int(ss*color_a[1]+(1-ss)*color_b[1])
		b=int(ss*color_a[2]+(1-ss)*color_b[2])
		return [r,g,b]
	
	def blue_neutral(self):
		self.binary_map_color_to_image(self.image['neutral'],self.colors['blue'])
		#print 'blue neutral'
		return m3b.res_continue
	
	def purple_neutral(self):
		self.binary_map_color_to_image(self.image['neutral'],self.colors['purple'])
		#print 'purple neutral'
		return m3b.res_continue
	
	
	def cool_neutral(self):
		rgb=self.color_cycle(self.colors['blue'],self.colors['green'])
		self.binary_map_color_to_image(self.image['neutral'],rgb)
		#print 'cool neutral'
		return m3b.res_continue
	
	def hot_smile(self):
		rgb=self.color_cycle(self.colors['red'],self.colors['pink'])
		self.binary_map_color_to_image(self.image['smile'],rgb)
		#print 'hot smile'
		return m3b.res_continue
	

# ######################################################	

class HeadBehaviors:
	def __init__(self):
		pass
	
	def start(self,proxy,bot,beh,csp=None):
	
		led_name=proxy.get_available_components('m3led_matrix_ec')[0]
		self.led=m3l.M3LedMatrixEc(led_name)
		self.led.enable_leds()
		proxy.publish_command(self.led)
		if csp is None:
			csp_name=proxy.get_available_components('m3head_s2csp_ctrl')[0]
			self.csp=m3csp.M3HeadS2CSPCtrl(csp_name)
		else:
			self.csp = csp
		proxy.publish_command(self.csp)
		proxy.publish_param(self.csp)
		if self.led is not None:
			self.led.enable_leds()
		#print 'Use facetracking (Ros services must be started in advance) [n]?'
		use_fd=False#m3t.get_yes_no('n')
		self.beh_look=LookBehaviors(bot,csp,beh,use_fd)
	
		self.beh_eyelids=EyelidBehaviors(bot,beh)
	
		beh.define_resource('led')
		beh.define_resource('look')
		beh.define_resource('look_roll')
		beh.define_resource('eyelids')

		self.beh_led=LedBehaviors(self.led,bot)
		beh.define_resource('led')
		beh.always('led','cool_neutral',priority=0,action=self.beh_led.cool_neutral)
		beh.random('led','blue_neutral',priority=1,action=self.beh_led.blue_neutral,chance=0.04,timeout=8.0, inhibit=8.0)
		beh.random('led','purple_neutral',priority=1,action=self.beh_led.purple_neutral,chance=0.04,timeout=8.0, inhibit=8.0)
		beh.random('led','hot_smile',priority=1,action=self.beh_led.hot_smile,chance=.04,timeout=8.0, inhibit=8.0)

	
		beh.always('eyelids','neutral',priority=0,action=self.beh_eyelids.neutral)
		beh.random('eyelids','blink', priority=1,action=self.beh_eyelids.blink,chance=0.01,timeout=1.0, inhibit=4.0)
		beh.random('eyelids','sleepy', priority=2,action=self.beh_eyelids.sleepy,chance=0.01,timeout=10.0, inhibit=25.0)
	
		beh.always('look','zero',priority=0,action=self.beh_look.zero)
		if use_fd:
			beh.whenever('look','facetrack',priority=3,action=self.beh_look.facetrack,cond=beh_look.face_detected,timeout=4.0,inhibit=0.0)
		beh.random('look','leftright',priority=2,action=self.beh_look.leftright,chance=0.1,timeout=5.0, inhibit=6.0)
		beh.random('look','random',priority=1,action=self.beh_look.random,chance=.1,timeout=4.0, inhibit=6.0)
	
	
		beh.always('look_roll','roll_zero',priority=0,action=self.beh_look.roll_zero)
		beh.random('look_roll','roll_backforth',priority=1,action=self.beh_look.roll_backforth,chance=0.07,timeout=5.0, inhibit=5.0)
	
		def stop(self):
			if self.led is not None:
				self.beh_led.stop()
			self.beh_eyelids.stop()
			self.beh_look.stop()

# ######################################################  	

if __name__ == '__main__':
	#Main creates bot and proxy, otherwise can be nested with other behavior script
	proxy = m3p.M3RtProxy()
	proxy.start()
	bot_name	= m3t.get_robot_name()
	bot			= m3f.create_component(bot_name)
	proxy.publish_param(bot) 
	proxy.subscribe_status(bot)
	proxy.publish_command(bot)
	proxy.make_operational_all()
	bot.set_motor_power_on()
	beh=m3b.M3BehaviorEngine(rate=.03)
	hb=HeadBehaviors()
	hb.start(proxy,bot,beh)
	ts=time.time()
	proxy.step() #Initialize data
	try:
		while True:
			proxy.step() 
			beh.step(verbose=True)
	except (KeyboardInterrupt,EOFError):
		pass
	proxy.step()
	print 'Exiting...'
	hb.stop()
	bot.set_motor_power_off()
	proxy.step()
	time.sleep(0.25)
	proxy.stop()

