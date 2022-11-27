import pygame
import sys
import numpy as np
from numpy import sin, cos, pi
from numpy.linalg import inv
from pygame.locals import *
from spring import spring

#Solver functions (Runge_Kutta4)

def G(y,t):
    x_d, theta_d, x, theta = y[0], y[1], y[2], y[3]

    a11, a12 = Mc+Mp, -Mp*l*cos(theta)
    a21, a22 = -cos(theta), l
    A = np.array([[a11,a12],[a21,a22]])

    b1 = -Mp*sin(theta)*l*theta_d**2
    b2 = g*sin(theta)
    B = np.array([b1,b2])
    F=np.array([0,0])
    #F[0] = F0

    accel = inv(A).dot(B)#+inv(A).dot(F)
    return np.array([accel[0], accel[1], x_d, theta_d])

def RK4_step(y, t, dt):
    k1 = G(y, t)
    k2 = G(y+0.5*k1*dt, t+0.5*dt)
    k3 = G(y+0.5*k2*dt, t+0.5*dt)
    k4 = G(y+k3*dt, t+dt)

    return dt * (k1 + 2*k2 + 2*k3 + k4) / 6

def angle2pos(theta):
    position = np.zeros(2)
    position[0] = int(l*sin(theta)*scale)
    position[1] = int(l*cos(theta)*scale)
    
    return position

#render functions and classes
def render():
    screen.fill(WHITE)
    render_statics()
    mass.render()
    pend.render()
    
def update(point2, point3):
    mass.update(point2)
    pend.update(point2, point3)
    
def render_statics():
    pygame.draw.line(screen, BLACK, (0, point1[1]), (1800, point1[1]),7)
    pygame.draw.circle(screen, BLACK, point1, 10)
    
class Mass():
    def __init__ (self, position, color, width, height):
        self.pos = position
        self.color = color
        self.w = width
        self.h = height
        self.left = self.pos[0] - self.w/2
        self.top = self.pos[1] - self.h/2
    def render(self):
        pygame.draw.rect(screen, self.color, (self.left, self.top, self.w, self.h))
    def update(self, position):
        self.pos = position
        self.left = self.pos[0] - self.w/2
        self.top = self.pos[1] - self.h/2

class Pendulum():
    def __init__ (self, position_attachment, position_pendulum, color, radius):
        self.pos = position_pendulum
        self.att = position_attachment
        self.color = color
        self.rad = radius
        self.left = self.pos[0]
        self.top = self.pos[1]
    def render(self):
        pygame.draw.circle(screen, BLACK, self.att, 5)
        pygame.draw.line(screen, BLACK, self.att, self.pos, 5)
        pygame.draw.circle(screen, self.color, (self.left, self.top), self.rad)
    def update(self, position_attachment, position_pendulum):
        self.pos = position_pendulum
        self.left = self.pos[0]
        self.top = self.pos[1]
        self.att = position_attachment

class Spring():
    def __init__(self, color, start, end, nodes, width, lead1, lead2):
        self.start = start
        self.end = end
        self.nodes = nodes
        self.width = width
        self.lead1 = lead1
        self.lead2 = lead2
        self.weight = 3
        self.color = color
    def update(self, start, end):
        self.start = start
        self.end = end
        self.x, self.y, self.p1, self.p2 = spring(self.start, self.end, self.nodes, self.width, self.lead1, self.lead2)
        self.p1 = (int(self.p1[0]), int(self.p1[1]))
        self.p2 = (int(self.p2[0]), int(self.p2[1]))
    def render(self):
        pygame.draw.line(screen, self.color, self.start, self.p1, self.weight)
        prev_point = self.p1
        for point in zip(self.x, self.y):
            pygame.draw.line(screen, self.color, prev_point, point, self.weight)
            prev_point = point
        pygame.draw.line(screen, self.color, self.p2, self.end, self.weight)

#Parameters
pendulum_length = 325
scale = 1
l = pendulum_length / scale
Mc = 5.0                        #cart mass
Mp = 1.0                        #pendulum mass
g = -9.81                        #gravitational force
t = 0.0                         #initial time
F0 = 0.0                        #initial force
delta_t = 0.1
y = np.array([0,0,0,2])         #initial conditions [cart speed, pendulum angular speed, cart position, pendulum angle]
point1 = (800,475)

#pygame and objects setup
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (150, 150, 150)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
screen = pygame.display.set_mode((0,0), pygame.FULLSCREEN)
screen.fill(WHITE)

mass = Mass(point1, RED, 180, 120)
pend = Pendulum(point1, (800,100), BLUE, 40)


#main loop

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
    pendulum_position = angle2pos(y[3])
    point2 = point1[0]+y[2], point1[1]
    point3 = point2[0]+pendulum_position[0], point2[1]+pendulum_position[1]

    update(point2,point3)
    render()
    
    t += delta_t
    y = y + RK4_step(y, t, delta_t)
    pygame.display.update()
