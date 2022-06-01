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
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

TIRE_RADIUS = 0.5
SUSPENSION_LENGTH = 1.75

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
    def __init__(self, world, chassis, start, suspension_length, tire_radius, kSpring=900.0, cDamp=25.0):
        self.world = world
        self.chassis = chassis
        # self.callback = RayCallback(chassis)
        self.ray_length = suspension_length + tire_radius
        self.suspension_length = suspension_length
        self.start = start
        self.end = b2Vec2(start.x, start.y - (suspension_length + tire_radius))

        self.cDamp = cDamp
        self.kSpring = kSpring

    def update(self):
        self.callback = RayCallback(self.chassis)
        # compute ray ends
        self.ray_start = self.chassis.GetWorldPoint(self.start)
        self.ray_end = self.chassis.GetWorldPoint(self.end)
        # print(f"start: {self.ray_start}")
        # print(f"end: {self.ray_end}")
        # do raycast
        self.world.RayCast(self.callback, self.ray_start, self.ray_end)

        if self.hasContact():
            # print(f"Got Hit! pos: {self.callback.point}, norm: {self.callback.normal}")
            self.computeSpringForce()

    
    def hasContact(self):
        return self.callback.fixture != None and (self.callback.fraction < 1.0 and self.callback.fraction > 0.0)

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
        print(f"v: {v}, n: {ct_norm}")

        up = self.chassis.GetWorldVector(b2Vec2(0.0, 1.0))
        scaleFactor = b2Vec2.dot(up, ct_norm)

        # compute distance (dx)
        dx = (1.0 - self.callback.fraction) * self.suspension_length

        fSpring = self.kSpring * dx * scaleFactor
        fDamp = self.cDamp * b2Vec2.dot(v, ct_norm)

        self.fSpring = fSpring - fDamp
        print(f"fSpring: {fSpring:.2f}, fDamp: {fDamp:.2f}")

        # final spring force?
        vSpring = b2Vec2(ct_norm) * self.fSpring
        print(-vSpring)

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
ground_body.fixtures[0].friction=0.9

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
car_shape = car_body.CreatePolygonFixture(box=(3.5, 1.25), density=1.5, friction=0.3)

front_wheel = Wheel(world, car_body, b2Vec2(1.75, 0.0), SUSPENSION_LENGTH, TIRE_RADIUS)
rear_wheel = Wheel(world, car_body, b2Vec2(-1.75, 0.0), SUSPENSION_LENGTH, TIRE_RADIUS)

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

def my_draw_ray(wheel):
    if (wheel.ray_start == None) or (wheel.ray_end == None):
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

def draw_ray_hit(pos, normal):
    pass

# --- main game loop ---

torque_mult = 0
bump_chassis = 0

running = True
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
    
    tire.ApplyTorque(torque_mult * 5.0, True)
    car_body.ApplyForceToCenter(b2Vec2(bump_chassis * 100.0, 0.0), True)

    # pygame.display.set_caption(f'{tire.}')
    pygame.display.set_caption(f'tire vel: {tire.linearVelocity.x:.2f}, {tire.linearVelocity.y:.2f}')

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
    for w in wheels:
        w.update()
        my_draw_ray(w)

    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
print('Done!')
