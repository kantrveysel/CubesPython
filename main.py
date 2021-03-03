from tkinter import *
from math import sin,cos,radians,sqrt,fabs,atan,tan
import numpy as np
from matplotlib import pyplot as pyp
import gc

## Pinned force problem
pyp.ion()

sign = lambda x: (x>0) - (x<0)

class Force:
	def __init__(self,x,y,fx,fy,pinned=False):
		self.x = x
		self.y = y
		self.fx = fx
		self.fy = fy
		self.t = np.array([[self.x, self.y], [self.fx, self.fy]])
		self.pinned = pinned
		self.ax = x
		self.ay = y
	def update(self):
		self.t = np.array([[self.x, self.y], [self.fx, self.fy]])
		if self.pinned != False:
			if self.ay != 0:
				mytheta = atan(self.ax/self.ay)
			else:
				mytheta = 0 if self.ax==0 else radians(180)
			self.x = cos(radians(-self.pinned.theta-45) + mytheta)*(sqrt(self.ax**2 + self.ay**2))
			self.y = sin(radians(-self.pinned.theta-45)+ mytheta)*(sqrt(self.ax**2 + self.ay**2))
			#print(self.x,self.y)
		

class Cube:
	def __init__(self,x,y,a,theta=-45,color="black"):
		self.x = x
		self.y = y
		self.a = a
		self.externalForces = [Force(0,0,0,0) for i in range(4)]
		self.forces = [i for i in self.externalForces]
		self.mass = .3*a**2
		self.J = self.mass*(a/2)**2
		self.theta = theta
		self.color = color
		self.lineerAcc = [0,0] 	# px/s2
		self.angularAcc = 0		# px/s2
		self.lineerVel = [0,0]
		self.angularVel = 0
		self.polygon = [x+a*cos(radians(-theta)),		y+a*sin(radians(-theta)),
						x+a*cos(radians(-theta+90)) ,	y+a*sin(radians(-theta+90)),
						x+a*cos(radians(-theta+180)),	y+a*sin(radians(-theta+180)),
						x+a*cos(radians(-theta+270)),	y+a*sin(radians(-theta+270))]

	def update(self):
		self.x += self.lineerVel[0]
		self.y += self.lineerVel[1]
		self.lineerVel[0] += self.lineerAcc[0]
		self.lineerVel[1] += self.lineerAcc[1]
		self.angularVel += self.angularAcc
		self.theta -= self.angularVel
		if self.theta>360:
			self.theta -=360
		elif self.theta<-360:
			self.theta+=360

	def isCollision(self,objects):
		for i in objects:
			obpolygon = np.array(i.polygon).reshape(4,2)
			for point in obpolygon:
				...
					

	def CalcFromForces(self):
		if len(self.forces)>0:
			self.TotalForce = np.array([i.t for i in self.forces]).sum(axis=0)[1]
			self.TotalMoment = sum([np.linalg.det(i.t) for i in self.forces])/10
			self.TotalMoment -= self.angularVel*500
			self.TotalForce[0] -= self.lineerVel[0]*500
			self.TotalForce[1] -= self.lineerVel[1]*500
			self.lineerAcc[0] += 	self.TotalForce[0]/self.mass
			self.lineerAcc[1] += 	self.TotalForce[1]/self.mass # a = F/m
			self.angularAcc = self.TotalMoment/self.J # alpha = M/J
	
	def draw(self,c):
		self.polygon = [self.x+self.a*cos(radians(-self.theta)),		self.y+self.a*sin(radians(-self.theta)),
						self.x+self.a*cos(radians(-self.theta+90)) ,	self.y+self.a*sin(radians(-self.theta+90)),
						self.x+self.a*cos(radians(-self.theta+180)),	self.y+self.a*sin(radians(-self.theta+180)),
						self.x+self.a*cos(radians(-self.theta+270)),	self.y+self.a*sin(radians(-self.theta+270))]
		canvas.create_polygon(self.polygon,fill=self.color,outline="black",width=2)

class Ground:
	def __init__(self,root,angle=0,height=20):
		self.root = root
		self.rgeometry = [640,480]
		self.angle = angle
		self.height = height
		
	def draw(self,canvas : "Tk.Canvas"):
		self.rgeometry = [int(root.geometry().split("x")[0]),int(root.geometry().split("x")[1].split("+")[0])]
		canvas.create_line(0, self.rgeometry[1]-self.height-sin(self.angle)*self.rgeometry[0], self.rgeometry[0], self.rgeometry[1]-self.height,width=3) # m = (y-y1)/(x-x1)
		
	
	def iscollision(self,objects):
		m = sin(self.angle)
		y1 = self.rgeometry[1]-self.height-sin(self.angle)*self.rgeometry[0] # y-y1 = m(x-x1)
		# m <= y/x
		for i in objects:
			polygon = np.array(i.polygon).reshape(4,2)
			noTouch = True
			z=0
			for u in polygon:
				if m <= (u[1]-y1)/u[0]:
					i.externalForces[z].x = u[0] - i.x
					i.externalForces[z].y = u[1] - i.y
					i.externalForces[z].fx = i.mass*0.1*sin(self.angle)
					i.externalForces[z].fy = -i.mass*0.1*cos(self.angle)
					z+=1
					noTouch = False
					#print(f"({u[0]} , {u[1]}) of object {hex(id(i))}")
			if(noTouch):
				for k in range(4):
					i.externalForces[k].x = 0
					i.externalForces[k].y = 0
					i.externalForces[k].fx = 0
					i.externalForces[k].fy = 0
					
root = Tk()
cube1 = Cube(320,320,64,color="#ffaa99")
cube2 = Cube(320,200,64,color="#99aaff")

canvas = Canvas(root, width = 640, height = 480, bg="white")
canvas.pack()

x,y = 320,200
a = 64
theta = -45
cube1.forces.append(Force(0,0,0,cube1.mass/10))
cube2.forces.append(Force(0,0,0,cube2.mass/10))

groundF = Force(0,0,0,0)
pinnedF = Force(32,1,0,-0*cube1.mass/10,pinned=cube1)
pinnedF2 = Force(-32,-1,0,-cube1.mass/20,pinned=cube1)
mouseF = Force(0,0,0,0)
cube1.forces.append(groundF)
cube1.forces.append(pinnedF)
cube1.forces.append(mouseF)

ground = Ground(root,radians(15))

def draw():
	root.after(50,draw)
	canvas.delete("all")
	ground.draw(canvas)
	cube1.draw(canvas)
	cube2.draw(canvas)
	[canvas.create_line(cube1.x+i.x,cube1.y+i.y,cube1.x+i.fx+i.x,cube1.y+i.fy+i.y,width=3,arrow=LAST) for i in cube1.forces]
	[canvas.create_line(cube2.x+i.x,cube2.y+i.y,cube2.x+i.fx+i.x,cube2.y+i.fy+i.y,width=3,arrow=LAST) for i in cube2.forces]

t=0
buffer =0

def loop():
	global t,buffer
	gc.collect()
	t+=1
	root.after(1,loop)
	cube1.CalcFromForces()
	#if(t>5):
	#	pyp.plot([t-1,t],[buffer, cube1.TotalMoment],color="black")
	buffer = cube1.TotalMoment
	cube2.CalcFromForces()
	ground.iscollision([cube1,cube2])
	cube1.isCollision([cube2])
	cube2.isCollision([cube1])
	cube1.update()
	cube2.update()
	[i.update() for i in cube1.forces]
	[i.update() for i in cube2.forces]

def mouse(e):
	if e.x > cube1.x-cube1.a and e.x < cube1.x+cube1.a and e.y > cube1.y-cube1.a and e.y < cube1.y+cube1.a:
		mouseF.x = e.x-cube1.x
		mouseF.y = e.y-cube1.y
	else:
		mouseF.fx = e.x-cube1.x
		mouseF.fy = e.y-cube1.y
	if e.num==3:
		mouseF.x,mouseF.y,mouseF.fx,mouseF.fy = (0,0,0,0)
	#print(e)
loop()
draw()
root.bind("<Button-1>",mouse)
root.bind("<Button-3>",mouse)
root.mainloop()
