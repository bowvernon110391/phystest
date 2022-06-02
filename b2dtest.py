#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
An attempt at some simple, self-contained pygame-based examples.
Example 02

In short:
One static body:
    + One fixture: big polygon to represent the ground
Two dynamic bodies:
    + One fixture: a polygon
    + One fixture: a circle
And some drawing code that extends the shape classes.

kne
"""
from math import sin,cos, sqrt
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE, KEYUP, K_a, K_d)

import Box2D  # The main library
# Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody)
from Box2D import b2RayCastCallback, b2Vec2, b2Vec2_zero

# --- constants ---
# Box2D deals with meters, but we want to display pixels,
# so define a conversion factor:
PPM = 20.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600

TIRE_RADIUS = 0.5
SUSPENSION_LENGTH = 1.75

# helper
def cross(v):
    return (v[1], -v[0])

def clamp(v, vmin, vmax):
    return min(max(v, vmin), vmax)

# custom callback
class RayCallback(b2RayCastCallback):
    def __init__(self, chassis):
        super().__init__()
        self.fixture = None
        self.chassis = chassis
    
    def ReportFixture(self, fixture, point, normal, fraction):
        # skip if it's the car chassis
        if fixture in self.chassis.fixtures:
            return 1.0

        # otherwise, record closest
        self.fixture = fixture
        self.point = point
        self.normal = normal
        self.fraction = fraction

        # print(self.fixture)
        # print(self.point)
        # print(self.normal)
        # print(fraction)
        return fraction

class Wheel:
    def __init__(self, world, chassis, start, suspension_length, tire_radius, wheelMass=10.0, kSpring=900.0, cDamp=25.0, friction=0.9):
        self.world = world
        self.chassis = chassis
        # self.callback = RayCallback(chassis)
        self.ray_length = suspension_length + tire_radius
        self.suspension_length = suspension_length
        self.start = start
        self.end = b2Vec2(start.x, start.y - (suspension_length + tire_radius))

        self.cDamp = cDamp
        self.kSpring = kSpring

        # wheel properties
        self.friction = 0.9
        self.realFriction = 0.0 # updated on contact
        self.tire_radius = tire_radius
        self.mass = wheelMass
        self.invMass = 1.0 / wheelMass
        self.inertia = 0.5 * wheelMass * tire_radius ** 2
        self.invInertia = 1.0 / self.inertia

        self.rotation = 0.0 # in radians
        self.angVel = 0.0 # in rad/sec
        self.torque = 0.0 # in Newton meter

        # for constraint resolution
        self.accumN = 0.0

    def applyTorque(self, t):
        self.torque += t

    def applyImpulse(self, p, pos, norm):
        # apply impulse to the chassis at specified position?
        self.chassis.ApplyLinearImpulse(p, pos, True)

        # apply angular impulse to the wheel directly?
        # compute the torque
        tangent = b2Vec2(cross(norm)) * self.tire_radius

        angularImpulse = b2Vec2.dot(p, tangent)
        # print(f"angular_impulse: {angularImpulse:.2f}")
        self.angVel += self.invInertia * angularImpulse

    def getWorldVel(self, pos, normal):
        v1 = self.chassis.GetLinearVelocityFromWorldPoint(pos)
        v2 = b2Vec2(cross(normal)) * self.tire_radius * self.angVel
        return v1 + v2

    def update(self):
        self.callback = RayCallback(self.chassis)
        # compute ray ends
        self.ray_start = self.chassis.GetWorldPoint(self.start)
        self.ray_end = self.chassis.GetWorldPoint(self.end)
        # print(f"start: {self.ray_start}")
        # print(f"end: {self.ray_end}")
        # do raycast
        self.world.RayCast(self.callback, self.ray_start, self.ray_end)

        # update fraction of rayhit
        self.fraction = 1.0 if not hasattr(self.callback, 'fraction') else self.callback.fraction
        # reset normal factor
        self.normalFactor = 1.0
        # print(f"fraction: {self.fraction:.2f}")

        # integrate velocity and position
        self.angVel += self.torque * self.invInertia * TIME_STEP
        self.rotation += self.angVel * TIME_STEP

        # clear torque
        self.torque = 0.0

        # clear accumulated Normal
        self.fSpring = 0.0

        if self.hasContact():
            # print(f"Got Hit! pos: {self.callback.point}, norm: {self.callback.normal}")
            self.computeSpringForce()

    
    def hasContact(self):
        return self.callback.fixture != None and (self.callback.fraction < 1.0 and self.callback.fraction > 0.0)

    def preSolve(self):
        # compute appropriate data for solver
        self.accumT = 0.0
        self.massT = 0.0

        # compute mass?
        if self.hasContact():
            ct_pos = self.callback.point
            ct_norm = self.callback.normal
            b2 = self.callback.fixture.body

            # compute relative vector for both bodies
            r1 = b2Vec2(ct_pos) - self.chassis.position
            r2 = b2Vec2(ct_pos) - b2.position

            rn1 = b2Vec2.dot(r1, ct_norm)
            rn2 = b2Vec2.dot(r2, ct_norm)

            # save some prop for later
            self.tangent = cross(ct_norm) #(ct_norm[1], -ct_norm[0])
            self.ct_pos = ct_pos

            # worldVel = self.getWorldVel(ct_pos, ct_norm)
            # print(f"vel: {worldVel}")

            # compute effective mass
            total_mass = 1.0 / self.chassis.mass
            # watch out for static object
            if b2.mass > 0.0:
                total_mass += 1.0 / b2.mass

            # add relative inertia from chassis (b1)
            total_mass += 1.0 / self.chassis.inertia * (b2Vec2.dot(r1, r1) - rn1 * rn1)
            # watch out for static object
            if b2.inertia > 0.0:
                total_mass += 1.0 / b2.inertia * (b2Vec2.dot(r2, r2) - rn2 * rn2)

            if total_mass > 0.0:
                self.massT = 1.0 / total_mass

        # print(f"massT: {self.massT:.2f}, norm_imp: {self.accumN:.2f}, friction: {self.realFriction:.2f}")

    def solve(self):
        # compute shit
        if not self.hasContact():
            return

        b2 = self.callback.fixture.body
        ct_point = self.callback.point
        ct_normal = self.callback.normal
        # can compute shit now
        v1 = b2Vec2(self.getWorldVel(ct_point, ct_normal))
        v2 = b2.GetLinearVelocityFromWorldPoint(ct_point)

        vt = b2Vec2.dot(v1-v2, self.tangent)
        # print(f"vel: {vt}")

        dPt = self.massT * -vt

        maxPt = self.accumN * self.realFriction
        # print(f"maxPT: {maxPt:.2f}")
        oldPt = self.accumT
        self.accumT = clamp(oldPt + dPt, -maxPt, maxPt)
        dPt = self.accumT - oldPt

        pt = b2Vec2(self.tangent) * dPt

        # print(f"dPt: {dPt:.2f}")
        # print(f"pt: {pt}")
        self.applyImpulse(pt, ct_point, ct_normal)
        b2.ApplyLinearImpulse(-pt, ct_point, True)        

    def computeSpringForce(self):
        # Fspring = k.x - c.v
        # grab data
        ct_pos = self.callback.point
        ct_norm = self.callback.normal
        b2 = self.callback.fixture.body
        
        # velocity of contacts?
        v1 = self.chassis.GetLinearVelocityFromWorldPoint(ct_pos)
        v2 = b2.GetLinearVelocityFromWorldPoint(ct_pos)

        v = (v1-v2)
        # print(f"v: {v}, n: {ct_norm}")

        up = self.chassis.GetWorldVector(b2Vec2(0.0, 1.0))
        scaleFactor = b2Vec2.dot(up, ct_norm)

        # recompute normalFactor
        if scaleFactor > Box2D.b2_epsilon:
            self.normalFactor = 1.0 / scaleFactor

        self.realFriction = sqrt(self.friction * self.callback.fixture.friction)
        # print(self.realFriction)

        # compute distance (dx)
        dx = (1.0 - self.callback.fraction) * self.suspension_length

        fSpring = self.kSpring * dx * scaleFactor
        fDamp = self.cDamp * b2Vec2.dot(v, ct_norm)

        self.fSpring = max(0.0, fSpring - fDamp)
        # print(f"fSpring: {fSpring:.2f}, fDamp: {fDamp:.2f}")
        self.accumN = self.fSpring * TIME_STEP

        # final spring force?
        vSpring = b2Vec2(ct_norm) * self.fSpring
        # print(-vSpring)

        self.chassis.ApplyForce(vSpring, ct_pos, True)
        b2.ApplyForce(-vSpring, ct_pos, True)

# --- pygame setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

# --- pybox2d world setup ---
# Create the world
world = world(gravity=(0, -10), doSleep=True)

wheels = []

# And a static body to hold the ground shape
ground_body = world.CreateStaticBody(
    position=(0, 0),
    shapes=polygonShape(box=(50, 1)),
)
ground_body.fixtures[0].friction=1.0

# Create a couple dynamic bodies
body = world.CreateDynamicBody(position=(1, 15))
circle = body.CreateCircleFixture(radius=0.5, density=1, friction=0.9)
circle.filterData.groupIndex=-2
# body.ApplyLinearImpulse(Box2D.b2Vec2(5, 0), body.transform * Box2D.b2Vec2_zero, True)
body.linearVelocity = Box2D.b2Vec2(10.0, -15.0)

tire = body
# print(circle)

# second tire (slippery)
body = world.CreateDynamicBody(position=(1, 15))
circle2 = body.CreateCircleFixture(radius=0.5, density=1, friction=0.0)
circle2.filterData.groupIndex=-2
body.linearVelocity = Box2D.b2Vec2(10.0, -15.0)

tire2 = body

# a box angled
body = world.CreateDynamicBody(position=(30, 45), angle=15)
box = body.CreatePolygonFixture(box=(2, 1), density=1, friction=0.3)

# a bigger box on the left
car_body = world.CreateDynamicBody(position=(5, 5), angle=0)
car_shape = car_body.CreatePolygonFixture(box=(3.5, 1.25), density=2.5, friction=0.3)

front_wheel = Wheel(world, car_body, b2Vec2(1.75, 0.0), SUSPENSION_LENGTH, TIRE_RADIUS)
rear_wheel = Wheel(world, car_body, b2Vec2(-1.75, 0.0), SUSPENSION_LENGTH, 0.6, 15)

wheels.append(front_wheel)
wheels.append(rear_wheel)

# print(body)

colors = {
    staticBody: (255, 255, 255, 255),
    dynamicBody: (127, 127, 127, 255),
}

# Let's play with extending the shape classes to draw for us.


def my_draw_polygon(polygon, body, fixture):
    vertices = [(body.transform * v) * PPM for v in polygon.vertices]
    vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]
    pygame.draw.polygon(screen, colors[body.type], vertices)
polygonShape.draw = my_draw_polygon


def my_draw_circle(circle, body, fixture):
    position = body.transform * circle.pos * PPM
    position = (position[0], SCREEN_HEIGHT - position[1])
    pygame.draw.circle(screen, colors[body.type], [int(
        x) for x in position], int(circle.radius * PPM))

    # line_end = body.GetWorldPoint(Box2D.b2Vec2(circle.radius * PPM, 0.0))

    # print(f"{position[0]:.2f}, {position[1]:.2f} | {line_end.x:.2f}, {line_end.y:.2f}")
    line_end = body.transform * Box2D.b2Vec2(circle.radius, 0) * PPM
    line_end = (line_end[0], SCREEN_HEIGHT - line_end[1])
    
    pygame.draw.line(screen, colors[0], [int(x) for x in position], [int(x) for x in line_end])
    # Note: Python 3.x will enforce that pygame get the integers it requests,
    #       and it will not convert from float.
circleShape.draw = my_draw_circle

def draw_wheel(pos, radius, rotation, color=(200, 0, 100, 255)):
    pygame.draw.circle(screen, color, pos, radius, width=1)
    # draw line
    # print(rotation)
    line_end = [pos[0] + radius * cos(rotation), (pos[1] - radius * sin(rotation))]
    # print(pos)
    # print(line_end)
    pygame.draw.line(screen, color, pos, line_end)

def my_draw_ray(wheel):
    if not hasattr(wheel, 'ray_start') or not hasattr(wheel, 'ray_end'):
        return

    # draw here
    pos1 = (wheel.ray_start.x * PPM, SCREEN_HEIGHT - wheel.ray_start.y * PPM)
    pos2 = (wheel.ray_end.x * PPM, SCREEN_HEIGHT - wheel.ray_end.y * PPM)
    # print(pos1)
    # print(pos2)
    # # pygame.draw.line(screen, (255, 0, 0, 255), [int(x) for x in pos1], [int(x) for x in pos2])
    pygame.draw.line(screen, (0, 127, 0, 255), pos1, pos2)
    pygame.draw.circle(screen, (125, 0, 0, 255), pos1, 4.0)
    pygame.draw.circle(screen, (125, 0, 127, 255), pos2, 4.0)

    # compute wheel position
    # the suspension is just this long
    suspension_fraction = wheel.suspension_length / (wheel.suspension_length + wheel.tire_radius)
    hub_start = wheel.fraction * suspension_fraction * wheel.normalFactor
    
    hub_pos = [int(v1 * (1.0-hub_start) + v2 * hub_start) for (v1, v2) in zip(pos1, pos2)]
    # print(hub_pos)
    draw_wheel(hub_pos, wheel.tire_radius * PPM, wheel.rotation)

# --- main game loop ---

torque_mult = 0
bump_chassis = 0

modes = [{
    'name': "Front Wheel Drive",
    'wheel': front_wheel
}, {
    'name': "Rear Wheel Drive",
    'wheel': rear_wheel
}]
mode = 0

running = True

last_tick = pygame.time.get_ticks() / 1000.0
accum_dt = 0.0

while running:
    bump_chassis = 0
    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            running = False

        if event.type == KEYDOWN and event.key == K_d:
            torque_mult -= 1
        if event.type == KEYUP and event.key == K_d:
            torque_mult += 1

        if event.type == KEYDOWN and event.key == K_a:
            torque_mult += 1
        if event.type == KEYUP and event.key == K_a:
            torque_mult -= 1

        if event.type == KEYDOWN and event.key == pygame.K_RIGHT:
            bump_chassis += 1
        if event.type == KEYDOWN and event.key == pygame.K_LEFT:
            bump_chassis -= 1

        if event.type == KEYUP and event.key == pygame.K_m:
            mode = (mode + 1) & 1
            
    
    # tire.ApplyTorque(torque_mult * 5.0, True)
    w = modes[mode]['wheel']
    t = torque_mult * 150.0
    w.applyTorque(t)
    car_body.ApplyLinearImpulse(b2Vec2(bump_chassis * 100.0, 0.0), car_body.position, True)

    pygame.display.set_caption(f"Mode: {modes[mode]['name']}, Torque: {t:.2f} Nm")

    # pygame.display.set_caption(f'{tire.}')
    # pygame.display.set_caption(f'tire vel: {tire.linearVelocity.x:.2f}, {tire.linearVelocity.y:.2f}')

    screen.fill((0, 0, 0, 0))
    # Draw the world
    for body in world.bodies:
        for fixture in body.fixtures:
            fixture.shape.draw(body, fixture)

    mx, my = pygame.mouse.get_pos()
    # print(f"mouse: {mx}, {my}")
    pygame.draw.circle(screen, (127, 0, 0, 255), (int(mx), int(my)), 20.0)

    # Make Box2D simulate the physics of our world for one step.
    world.Step(TIME_STEP, 10, 10)

    # wheel sim
    current_tick = pygame.time.get_ticks() / 1000.0
    delta = current_tick - last_tick
    accum_dt += delta
    last_tick = current_tick

    # print(f"delta: {delta:.2f}")

    while accum_dt >= TIME_STEP:
        accum_dt -= TIME_STEP
        
        for w in wheels:
            w.update()

        # pre solve?
        for w in wheels:
            w.preSolve()

        # solve it
        for i in range(10):
            for w in wheels:
                w.solve()

    # wheel render
    for w in wheels:
        my_draw_ray(w)

    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
print('Done!')
