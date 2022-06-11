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
from math import radians, sin,cos, sqrt
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
SCREEN_WIDTH, SCREEN_HEIGHT = 1024, 600

TIRE_RADIUS = 0.5
SUSPENSION_LENGTH = 1.35
STATIC_TORQUE = 7200
BUMP_IMPULSE = 1500
ACCUMULATE = True   # use accumulation of impulse to converge faster
SLOP = 0.02 # 2 cm slop

# helper
def cross(v):
    return (v[1], -v[0])

def clamp(v, vmin, vmax):
    return min(max(v, vmin), vmax)

# abstract joint
class Joint:
    def solve(self):
        pass

    def preSolve(self):
        pass

class Updatable:
    def update(self):
        pass

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

class Wheel(Updatable, Joint):
    def __init__(self, world, chassis, start, suspension_length, tire_radius, wheelMass=24.0, kSpring=900.0, cDamp=50.0, friction=0.9):
        self.world = world
        self.chassis = chassis
        # self.callback = RayCallback(chassis)
        self.ray_length = suspension_length + tire_radius
        self.suspension_length = suspension_length
        self.start = start
        self.end = b2Vec2(start.x, start.y - (suspension_length + tire_radius))

        self.cDamp = cDamp
        self.kSpring = kSpring
        self.lastGroundObject = None

        # wheel properties
        self.friction = friction
        self.realFriction = 0.0 # updated on contact
        self.tire_radius = tire_radius
        self.mass = wheelMass
        self.invMass = 1.0 / wheelMass
        self.inertia = 0.5 * wheelMass * tire_radius ** 2
        self.invInertia = 1.0 / self.inertia

        # store real inertia, so it can be restored later
        self.realInvInertia = self.invInertia

        self.rotation = 0.0 # in radians
        self.angVel = 0.0 # in rad/sec
        self.old_angular_vel = 0.0  # for constraint solver
        self.torque = 0.0 # in Newton meter

        # for constraint resolution
        self.accumN = 0.0
        self.accumT = 0.0

        # for debug
        self.debug_ground_vel = (0.0, 0.0)
        self.debug_hub_vel = (0.0, 0.0)
        self.debug_patch_vel = 0.0

        self.debug_last_impulse = (0.0, 0.0)
        self.debug_last_angular_impulse = 0.0
        self.debug_max_angular_impulse = 0.0

    def handbrake(self, activate=True):
        self.invInertia = 0.0 if activate else self.realInvInertia
        self.angVel = 0.0 if activate else self.angVel

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

        if self.invInertia > 0.0:
            self.debug_max_angular_impulse = -self.angVel / self.invInertia
        else:
            self.debug_max_angular_impulse = 0.0

        self.angVel += self.invInertia * angularImpulse

        self.debug_last_impulse = p
        self.debug_last_angular_impulse = angularImpulse

    def getWorldVel(self, pos, normal, use_old_vel=False):
        v1 = self.chassis.GetLinearVelocityFromWorldPoint(pos)
        v2 = b2Vec2(cross(normal)) * self.tire_radius * (self.old_angular_vel if use_old_vel else self.angVel)
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

        # clear accumulated Normal
        self.fSpring = 0.0

        if self.hasContact():
            # if the last ground object is different, empty accumulator
            groundObject = self.callback.fixture.body
            if groundObject != self.lastGroundObject:
                # print(f"Different ground!, n({self.callback.normal[0]:.2f}, {self.callback.normal[1]:.2f})")
                self.lastGroundObject = groundObject
                self.accumT = 0.0
                self.accumN = 0.0
            # print(f"Got Hit! pos: {self.callback.point}, norm: {self.callback.normal}")
            self.computeSpringForce()
        else:
            # no contact? clear accumulator
            self.accumT = 0.0
            self.accumN = 0.0

        # integrate velocity and position
        self.old_angular_vel = self.angVel
        self.angVel += self.torque * self.invInertia * TIME_STEP

        # clear torque
        self.torque = 0.0

    def updatePosition(self):
        self.rotation += self.angVel * TIME_STEP
        # self.angVel *= 0.8
    
    def hasContact(self):
        return self.callback.fixture != None and (self.callback.fraction < 1.0 and self.callback.fraction > 0.0)

    def preSolve(self):
        # compute appropriate data for solver
        # self.accumT = 0.0
        self.massT = 0.0
        self.realFriction = 0.0

        self.debug_patch_vel = 0.0
        self.debug_hub_vel = (0.0, 0.0)

        # compute mass?
        if self.hasContact():
            self.realFriction = sqrt(self.friction * self.callback.fixture.friction)

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

            # let's add tire mass and inertia?
            total_mass += self.invMass
            total_mass += self.invInertia * self.tire_radius * self.tire_radius

            if total_mass > 0.0:
                self.massT = 1.0 / total_mass

            # apply old impulse
            if ACCUMULATE:
                # b1 = self.chassis
                b2 = self.lastGroundObject

                ct_point = self.callback.point
                ct_normal = self.callback.normal
                
                # where's the direction?
                pt = b2Vec2(self.tangent) * self.accumT

                self.applyImpulse(pt, ct_point, ct_normal)
                b2.ApplyLinearImpulse(-pt, ct_point, True)  

                # print(f"accumT: {self.accumT:.2f}")


        # print(f"massT: {self.massT:.4f}")

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

        dv = v1-v2
        self.debug_ground_vel = dv

        vt = b2Vec2.dot(dv, self.tangent)

        # print(f"target_vel: {vt}")

        dPt = self.massT * -vt
        # print(f"maxPT: {maxPt:.2f}")

        if not ACCUMULATE:
            maxPt = self.accumN * self.realFriction
            # not using accumulator, just simple clamp
            dPt = clamp(dPt, -maxPt, maxPt)
        else:
            maxPt = self.accumN / TIME_STEP * self.realFriction
            # otherwise, clamp the final impulse
            oldPt = self.accumT
            self.accumT = clamp(oldPt + dPt, -maxPt, maxPt)
            dPt = self.accumT - oldPt

        pt = b2Vec2(self.tangent) * dPt

        # print(f"dPt: {dPt:.2f}")
        # print(f"pt: {pt}")
        self.applyImpulse(pt, ct_point, ct_normal)
        b2.ApplyLinearImpulse(-pt, ct_point, True)        

        self.debug_hub_vel = self.chassis.GetLinearVelocityFromWorldPoint(ct_point)
        self.debug_patch_vel = self.angVel * self.tire_radius

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
        # print(self.realFriction)

        # compute distance (dx)
        dx = (1.0 - self.callback.fraction) * self.suspension_length - SLOP

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

# this is like a tank tracks. force both wheels to 
# run at same linear velocity (different angular velocity tho)
class WheelTrack(Joint):
    def __init__(self, w1:Wheel, w2:Wheel):
        self.w1 = w1
        self.w2 = w2

        self.active = True

        self.mass = 0.0

    def preSolve(self):
        if not self.active:
            return

        self.mass = 0.0
        if self.w1.invInertia > 0.0:
            self.mass += self.w1.invInertia * self.w1.tire_radius * self.w1.tire_radius
        if self.w2.invInertia > 0.0:
            self.mass += self.w2.invInertia * self.w2.tire_radius * self.w2.tire_radius

        if self.mass > 0.0:
            self.mass = 1.0 / self.mass

    def solve(self):
        if not self.active:
            return
        # compute delta v
        v1 = self.w1.angVel * self.w1.tire_radius
        v2 = self.w2.angVel * self.w2.tire_radius

        # the lambda
        lmb = -self.mass * (v1 - v2)

        # the magnitude is different
        p1 = self.w1.tire_radius * lmb
        p2 = -self.w2.tire_radius * lmb

        # apply both impulse
        self.w1.angVel += self.w1.invInertia * p1
        self.w2.angVel += self.w2.invInertia * p2

# Differential. Represented as updatable joint
class Differential(Updatable, Joint):
    def __init__(self, w1:Wheel, w2:Wheel):
        # keep tracks of our angular velocity
        self.angVel = 0.0
        # combined inertia of both wheels
        self.w1 = w1
        self.w2 = w2
        self.invInertia = 1.0 / (1.0/w1.invInertia + 1.0/w2.invInertia)
        self.torque = 0.0
        self.massT = 0.0 # the constraint mass
        self.active = True

    def slip(self):
        return self.w1.angVel - self.w2.angVel

    def applyTorque(self, t):
        self.torque += t

    # this is because the wheel could be locked which will have infinite inertia
    # def updateInertia(self):
    #     if not self.active:
    #         return

    #     total_inertia = 0.0
    #     self.invInertia = 0.0   # assume infinite inertia
    #     wheels = [self.w1, self.w2]

    #     for w in wheels:
    #         if w.invInertia > 0.0:
    #             total_inertia += 1.0 / w.invInertia

    #     if total_inertia > 0.0:
    #         self.invInertia = 1.0 / total_inertia

    def update(self):
        if not self.active:
            return
        # recompute inertia just in case
        # self.updateInertia()
        # update angular velocity
        self.angVel += self.invInertia * self.torque * TIME_STEP
        # zero out torque
        self.torque = 0.0

    def preSolve(self):
        if not self.active:
            return
        self.massT = 4.0 * self.invInertia + self.w1.invInertia + self.w2.invInertia
        if self.massT > 0.0:
            self.massT = 1.0 / self.massT
        else:
            self.massT = 0.0

    def solve(self):
        if not self.active:
            return
        
        dv = 2.0 * self.angVel - self.w1.angVel - self.w2.angVel

        lmb = -self.massT * dv

        # compute impulse
        ptSelf = 2 * lmb
        ptW1 = -lmb
        ptW2 = -lmb

        # apply impulse
        self.angVel += self.invInertia * ptSelf
        self.w1.angVel += self.w1.invInertia * ptW1
        self.w2.angVel += self.w2.invInertia * ptW2

class SpeedSensitiveLSD(Differential):
    def __init__(self, w1: Wheel, w2: Wheel, max_slip: float):
        super().__init__(w1, w2)
        self.max_slip = max_slip
        self.effMass = 0.0 # different from parent's mass

    def preSolve(self):
        # parent's compute its own effective mass
        super().preSolve()

        # we compute different effective mass than parent's
        self.effMass = self.w1.invInertia + self.w2.invInertia
        # compute the real shiet
        if self.effMass > 0.0:
            self.effMass = 1.0 / self.effMass
        else:
            self.effMass = 0.0

    # only different in solving it I guess
    def solve(self):
        if not self.active:
            return
        # first, solve the differential as usual
        super().solve()
        # next, remove the speed difference up to a certain limit
        dv = self.w1.angVel - self.w2.angVel
        # clamp lambda (might need to multiply with current eff mass?)
        allowed = clamp(dv, -self.max_slip, self.max_slip)

        lmb = -self.effMass * (dv - allowed)

        self.w1.angVel += self.w1.invInertia * lmb
        self.w2.angVel += self.w2.invInertia * -lmb

# --- pygame setup ---
pygame.font.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()
default_font = pygame.font.SysFont('Arial', 16)

def draw_text(text, pos, color=(255,255,255)):
    img = default_font.render(text, True, color)
    screen.blit(img, pos)

# --- pybox2d world setup ---
# Create the world
world = world(gravity=(0, -10), doSleep=True)

wheels = []

# updatables
updatables = []
# joints
joints = []

def addWheel(w:Wheel):
    updatables.append(w)
    joints.append(w)
    wheels.append(w)

def addDifferential(d:Differential):
    updatables.append(d)
    joints.append(d)

# And a static body to hold the ground shape
ground_body = world.CreateStaticBody(
    position=(0, 0),
    shapes=polygonShape(box=(50, 1)),
)
ground_body.fixtures[0].friction=1.0

ground_body = world.CreateStaticBody(
    position=(40, 0),
    angle=radians(20),
    shapes=polygonShape(box=(50, 1))
)
ground_body.fixtures[0].friction=0.5

ground_body = world.CreateStaticBody(
    position=(10, 0),
    angle=radians(-20),
    shapes=polygonShape(box=(50, 1))
)
ground_body.fixtures[0].friction=0.4

# Create a couple dynamic bodies
body = world.CreateDynamicBody(position=(2, 15))
# circle = body.CreateCircleFixture(radius=0.5, density=100, friction=0.9)
box = body.CreatePolygonFixture(box=(0.5,0.5), density=1.0, friction=0.9)
# circle.filterData.groupIndex=-2
# body.ApplyLinearImpulse(Box2D.b2Vec2(5, 0), body.transform * Box2D.b2Vec2_zero, True)
# body.linearVelocity = Box2D.b2Vec2(10.0, -15.0)

tire = body
# print(circle)

# second tire (slippery)
body = world.CreateDynamicBody(position=(1, 15))
# circle2 = body.CreateCircleFixture(radius=0.5, density=50, friction=0.2)
box = body.CreatePolygonFixture(box=(0.5,0.5), density=1.0, friction=0.2)
# circle2.filterData.groupIndex=-2
# body.linearVelocity = Box2D.b2Vec2(10.0, -15.0)

tire2 = body

# a box angled
# body = world.CreateDynamicBody(position=(30, 45), angle=15)
# box = body.CreatePolygonFixture(box=(2, 1), density=110, friction=0.3)

# a bigger box on the left
car_body = world.CreateDynamicBody(position=(5, 5), angle=0)
car_shape = car_body.CreatePolygonFixture(box=(3.5, 1.25), density=80.5, friction=0.3)

front_wheel = Wheel(world, car_body, b2Vec2(1.75, 0.0), SUSPENSION_LENGTH, TIRE_RADIUS, 25.0, 95000, 9500, 0.8)
rear_wheel = Wheel(world, car_body, b2Vec2(-1.75, 0.0), SUSPENSION_LENGTH, TIRE_RADIUS, 25.0, 95000, 9500, 0.8)

# wheels.append(front_wheel)
# wheels.append(rear_wheel)
addWheel(front_wheel)
addWheel(rear_wheel)

diff = Differential(front_wheel, rear_wheel)
diff.active = False

addDifferential(diff)

lsd = SpeedSensitiveLSD(front_wheel, rear_wheel, 50.0)
lsd.active = False

addDifferential(lsd)

coupler = WheelTrack(front_wheel, rear_wheel)
coupler.active = False
# print(body)
joints.append(coupler)

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

def draw_contact(pos, normal):
    pygame.draw.line()

def my_draw_ray(wheel):
    if not hasattr(wheel, 'ray_start') or not hasattr(wheel, 'ray_end'):
        return

    # refresh ray start
    wheel.ray_start = wheel.chassis.GetWorldPoint(wheel.start)
    wheel.ray_end = wheel.chassis.GetWorldPoint(wheel.end)

    # draw here
    pos1 = (wheel.ray_start.x * PPM, SCREEN_HEIGHT - wheel.ray_start.y * PPM)
    pos2 = (wheel.ray_end.x * PPM, SCREEN_HEIGHT - wheel.ray_end.y * PPM)

    ct_point = [int(v1 * (1.0-wheel.fraction) + v2 * wheel.fraction) for (v1, v2) in zip(pos1, pos2)]
    # print(pos1)
    # print(pos2)
    # # pygame.draw.line(screen, (255, 0, 0, 255), [int(x) for x in pos1], [int(x) for x in pos2])
    pygame.draw.line(screen, (0, 127, 0, 255), pos1, pos2)
    pygame.draw.circle(screen, (125, 0, 0, 255), pos1, 4.0)
    pygame.draw.circle(screen, (125, 0, 127, 255), pos2, 4.0)

    # contact point
    pygame.draw.circle(screen, (125, 255, 0, 255), ct_point, 4.0)
    # contact normal
    if wheel.hasContact():
        ct_normal = (wheel.callback.normal[0] * PPM, -wheel.callback.normal[1] * PPM)
        pygame.draw.line(screen, (180, 200, 0, 255), ct_point, [x+y for (x,y) in zip(ct_point, ct_normal)])

    # compute wheel position
    # the suspension is just this long
    suspension_fraction = wheel.suspension_length / (wheel.suspension_length + wheel.tire_radius)
    
    suspension_dir = (b2Vec2(pos1) - b2Vec2(pos2))
    suspension_dir = suspension_dir / suspension_dir.length
    # print(sus)

    normal_factor = wheel.normalFactor if wheel.normalFactor < 1.414 else 1.414
    
    hub_pos = b2Vec2(ct_point) + (suspension_dir * wheel.tire_radius * normal_factor * PPM)

    # print(f"NF: {wheel.normalFactor:.2f}")    
    
    # hub_pos = [int(v1 * (1.0-hub_start) + v2 * hub_start) for (v1, v2) in zip(pos1, pos2)]
    # print(hub_pos)
    draw_wheel([int(x) for x in hub_pos], wheel.tire_radius * PPM, wheel.rotation)

# --- main game loop ---

torque_mult = 0
bump_chassis = 0

modes = [
    {
        'name': "Front Wheel Drive",
        'wheel': front_wheel,
    },
    {
        'name': "Rear Wheel Drive",
        'wheel': rear_wheel
    },
    {
        'name': "Four Wheel Drive (locked diff)",
        'wheel': rear_wheel
    },
    {
        'name': "AWD with open differential",
        'wheel': diff
    },
    {
        'name': f"Speed Sensitive Limited Slip Differential",
        'wheel': lsd
    }
]
mode = 0

running = True

step_mode = False
step_forward = False

last_tick = pygame.time.get_ticks() / 1000.0
accum_dt = 0.0

while running:
    # clear forces
    bump_chassis = 0
    # activate/deactivate shit depending on mode
    coupler.active = True if mode == 2 else False
    diff.active = True if mode == 3 else False
    lsd.active = True if mode == 4 else False
    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            running = False

        if event.type == KEYUP and event.key == pygame.K_F1:
            step_mode = not step_mode

        if event.type == KEYUP and event.key == pygame.K_SPACE:
            if step_mode:
                step_forward = True
            else:
                step_mode = True

        if event.type == KEYDOWN and event.key == K_d:
            torque_mult -= 1
        if event.type == KEYUP and event.key == K_d:
            torque_mult += 1

        if event.type == KEYDOWN and event.key == K_a:
            torque_mult += 1
        if event.type == KEYUP and event.key == K_a:
            torque_mult -= 1

        if event.type == KEYDOWN and event.key == pygame.K_s:
            rear_wheel.handbrake(True)
            # front_wheel.handbrake(True)
        if event.type == KEYUP and event.key == pygame.K_s:
            rear_wheel.handbrake(False) 
            # front_wheel.handbrake(False)

        if event.type == KEYDOWN and event.key == pygame.K_RIGHT:
            bump_chassis += 1
        if event.type == KEYDOWN and event.key == pygame.K_LEFT:
            bump_chassis -= 1

        if event.type == KEYUP and event.key == pygame.K_m:
            mode = (mode + 1) % len(modes)

        if event.type == KEYUP and event.key == pygame.K_F2:
            ACCUMULATE = not ACCUMULATE

        # if event.type == KEYUP and event.key == pygame.K_c:
        #     coupler.active = not coupler.active
            
    
    # tire.ApplyTorque(torque_mult * 5.0, True)
    w = modes[mode]['wheel']
    t = torque_mult * STATIC_TORQUE
    # apply to wheel or the differential
    # w.applyTorque(t)
    w.applyTorque(t)
    car_body.ApplyLinearImpulse(b2Vec2(bump_chassis * BUMP_IMPULSE, 0.0), car_body.position, True)

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

    # wheel sim
    current_tick = pygame.time.get_ticks() / 1000.0
    delta = current_tick - last_tick
    accum_dt += delta
    last_tick = current_tick

    # print(f"delta: {delta:.2f}")
    if (step_mode and step_forward) or not step_mode:
        step_forward = False

        # for w in wheels:
        #     w.update()
        for u in updatables:
            u.update()

        # for w in wheels:
        #     w.preSolve()
        for j in joints:
            j.preSolve()

        # print("solve_start--------------------------------------------------")
        for i in range(10):
            # for w in wheels:
            #     w.solve()
            for j in joints:
                j.solve()

        for w in wheels:
            w.updatePosition()

        # Make Box2D simulate the physics of our world for one step.
        world.Step(TIME_STEP, 10, 10)

    # while accum_dt >= TIME_STEP:
    #     accum_dt -= TIME_STEP
        
        

        # pre solve?
        

        # solve it

        # update position
        

    # wheel render
    for w in wheels:
        my_draw_ray(w)

    # draw step_mode

    # draw status of all wheel
    pos_y = 22
    pos_x = 4

    accumtext = 'ON' if ACCUMULATE else 'OFF'
    draw_text(f"chassis: mass {car_body.mass:.2f}, ACCUMULATE: {accumtext}", (pos_x, 4))
    for (id, w) in enumerate(wheels):
        draw_text(f"Wheel_{id} : last_ang_p {w.debug_last_angular_impulse:.2f}, max {w.debug_max_angular_impulse:.2f}, angVel({w.angVel:.2f}), gnd_vel:({w.debug_ground_vel[0]:.2f}, {w.debug_ground_vel[1]:.2f}), W: {w.fSpring:.2f}, hub_vel: {w.debug_hub_vel[0]:.4f}, {w.debug_hub_vel[1]:.4f}, patch_vel: {w.debug_patch_vel:.4f}, last_p: {w.debug_last_impulse[0]:.2f}, {w.debug_last_impulse[1]:.2f}", (pos_x, pos_y))
        pos_y += 16

    if mode == 3:
        draw_text(f"open differential: w({diff.angVel:.2f}), invInertia({diff.invInertia:.2f}), slip({diff.slip():.2f})", (pos_x, pos_y))

    if mode == 4:
        draw_text(f"Speed Sensitive LSD: w({lsd.angVel:.2f}), invInertia({lsd.invInertia:.2f}), slip({lsd.slip():.2f}), max_slip({lsd.max_slip:.2f})", (pos_x, pos_y))

    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
print('Done!')
