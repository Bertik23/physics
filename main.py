import pygame as pg
from math import sqrt, sin, cos, pi
import time

# COLORS
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
LIME = (0, 255, 0)
# END COLORS

# SETTINGS
size = (600, 600)
# END SETTINGS

# PHYSICS SETTINGS
G = 6.67430 * 10**-11  # m^3kg^-1s^-2
M_E = 5.9736 * 10**24  # kg
R_Z = 6378000  # m
# END PHYSICS SETTINGS

deltaT = 0


def posNegZer(n):
    if n == 0:
        return 0
    elif n < 0:
        return -1
    elif n > 0:
        return 1
    raise ValueError


def getPointOnLine(xy1, xy2, n):
    x1, y1 = xy1
    x2, y2 = xy2
    d = sqrt((x2-x1)**2 + (y2 - y1)**2)  # distance
    r = n / d  # segment ratio

    x3 = r * x2 + (1 - r) * x1  # find point that divides the segment
    y3 = r * y2 + (1 - r) * y1  # into the ratio (1-r):r
    return (x3, y3)


def gravitation(mass1, mass2, r):
    return (G*mass1*mass2)/r**2


class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        if type(other) != Vector2:
            raise NotImplementedError("Ony addition of Vector2 supported")
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        if type(other) != Vector2:
            raise NotImplementedError("Ony addition of Vector2 supported")
        return self + (-other)

    def __neg__(self):
        return Vector2(-self.x, -self.y)

    def __abs__(self):
        return sqrt(self.x**2 + self.y**2)

    def __mul__(self, other):
        if type(other) in [int, float]:
            return Vector2(self.x * other, self.y * other)

    def __truediv__(self, other):
        if type(other) in [int, float]:
            return Vector2(self.x / other, self.y / other)

    def __str__(self):
        return f"Vector2({self.x}, {self.y})"


class CollisionBox:
    def __init__(self, type, points):
        self.type = type
        self.points = points


class RigidBody:
    def __init__(
            self,
            world,
            position=(0, 0),
            mass=1,
            moveable=True,
            collisionBox=None
    ):
        self.world = world
        self.mass = mass
        self.moveable = moveable
        self.collisionBox = collisionBox
        self.position = position
        self.rotation = 0
        self.velocity = Vector2(0, 0)

    @property
    def x(self):
        return self.position[0]

    @property
    def y(self):
        return self.position[1]

    def gravity(self):
        Fg_final = Vector2(0, 0)
        for o in self.world:
            if o == self:
                continue
            Fg_size = gravitation(
                self.mass,
                o.mass,
                abs(Vector2(self.x, self.y)-Vector2(o.x, o.y))
            )
            Fg = Vector2(*getPointOnLine(self.position, o.position, Fg_size))
            Fg_final += Fg
            print(getPointOnLine(self.position, o.position, Fg_size))
            print("FG", Fg_final, Fg_size, self.position, o.position)
        self.velocity += Fg_final*deltaT/self.mass

    def movement(self):
        if self.moveable:
            print(self.velocity)
            self.position = (
                self.position[0] + self.velocity.x * deltaT,
                self.position[1] + self.velocity.y * deltaT,
            )


class World:
    def __init__(self):
        self.__id = 0
        self.__objects = {}

    @property
    def objects(self):
        for i in self:
            yield i

    def addObject(self, type, *args, **kwargs):
        self.__objects[self.__id] = type(self, *args, **kwargs)
        self.__id += 1
        return self.__id - 1

    def __getitem__(self, item):
        print(item, self.__objects)
        return self.__objects[item]

    def __iter__(self):
        for i in self.__objects:
            yield self.__objects[i]

    def draw(self, surface):
        for i in self:
            i.draw(surface)

    def gravity(self):
        for i in self:
            i.gravity()

    def movement(self):
        for i in self:
            i.movement()


class Rectangle(RigidBody):
    def __init__(
            self,
            world,
            position,
            size,
            mass=1,
            moveable=True,
            color=WHITE
    ):
        collisionbox = CollisionBox(
            "rectangle",
            [
                [-size[0]/2, -size[1]/2],
                [-size[0]/2, size[1]/2],
                [size[0]/2, size[1]/2],
                [size[0]/2, -size[1]/2]
            ]
        )
        self.color = color
        super().__init__(world, position, mass, moveable, collisionbox)

    def draw(self, surface):
        pg.draw.polygon(surface, self.color, self.getPoints())
        self.rotation += 0.001

    def getPoints(self):
        return [
            (
                i[1]*sin(self.rotation) - i[0]*cos(self.rotation) + self.x,
                i[1]*cos(self.rotation) + i[0]*sin(self.rotation) + self.y
            ) for i in self.collisionBox.points
        ]


if __name__ == '__main__':
    pg.init()

    world = World()

    world.addObject(Rectangle, (0, 0), (60, 100))

    world.addObject(Rectangle, (500, 500), (60, 100))

    display = pg.display.set_mode(size, pg.RESIZABLE)

    running = True

    while running:
        frameStartTime = time.time()
        for u in pg.event.get():
            if u.type == pg.QUIT:
                running = False
            elif u.type == pg.VIDEORESIZE:
                size = (u.w, u.h)
                display = pg.display.set_mode(size, pg.RESIZABLE)

        display.fill(BLACK)

        world.draw(display)
        world.gravity()
        world.movement()

        pg.display.update()
        deltaT = time.time() - frameStartTime
