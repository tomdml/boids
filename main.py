import bisect
import math
import random
import profile

import pygame

from utils import *
from boid import Boid
from config import Config
from vector import Vector2D


def draw_scene(window, boids, shape=Config.BOID_SHAPE, draw_fov=Config.DRAW_FOV):
    """Draw the list of Boids to the screen.

    Arguments:
    window -- Pygame window. Required.
    boids -- List of Boid objects. Required.

    Keyword Arguments:
    shape -- Shape of Boid objects to be drawn.
    draw_fov -- Boolean representing whether to draw FOV lines around boids[0].
    """

    window.fill((30, 30, 30))

    for boid in boids:  # Draw each boid.
        if shape == 'DOT':
            pygame.draw.circle(window, boid.color, boid.pos, 1)
        elif shape == 'LINE':
            front, back = boid.get_line_points()
            pygame.draw.line(window, boid.color, front, back)
        elif shape == 'ARR':
            points = boid.get_arrow_points()
            pygame.draw.polygon(window, boid.color, points)
        else:
            raise ValueError('Unknown value for keyword argument `shape`')

    if draw_fov:
        pygame.draw.arc(window, (50, 50, 50), *boids[0].get_fov_arc_points(), 1)
        pygame.draw.lines(window, (50, 50, 50), False, boids[0].get_fov_line_points(), 1)

    pygame.display.flip()  # Flush display buffer


def preprocessSearch(boids):
    """Given a list of Boids, preprocesses the list by producing lists of boids sorted by xpos and ypos."""

    boidsByX = sorted(boids, key=lambda boid: boid.pos.x)
    boidsByY = sorted(boids, key=lambda boid: boid.pos.y)

    # We return the key-lists separately for binary searching as the bisect module does not support key arguments.
    boidKeysByX = [boid.pos.x for boid in boidsByX]
    boidKeysByY = [boid.pos.y for boid in boidsByY]

    return (boidsByX, boidKeysByX, boidsByY, boidKeysByY)


def candidatesInRange(sortedBoids, sortedKeys, coord, limit, radius):
    """Utilize binary search to produce a list of all boids within range r of a coordinate c.

    Arguments:
    sortedBoids - List of Boid objects, sorted by x or y coordinate.
    sortedKeys - List of Integers corresponding to sortedBoids x or y coordinates, for binary search.
    c - x or y coordinate of target Boid
    limit - maximum size of the screen in x or y direction
    r - View radius
    """

    left_bound, right_bound = coord - radius, coord + radius

    if 0 < left_bound and right_bound < limit:
        invert = False
    else:
        invert = True
        left_bound %= limit
        right_bound %= limit

    # We binary search on the list of KEYS as bisect does not support a custom key function.
    left_index = bisect.bisect_left(sortedKeys, left_bound)
    right_index = bisect.bisect_right(sortedKeys, right_bound)

    # We then index the list of BOIDS by the indices found when binary searching the keys.
    if invert:
        return sortedBoids[:right_index + 1] + sortedBoids[left_index:]
    else:
        return sortedBoids[left_index:right_index + 1]


def main():
    window = pygame.display.set_mode((Config.SCREEN_X, Config.SCREEN_Y))
    pygame.display.set_caption('Boids')

    clock = pygame.time.Clock()

    boids = [Boid() for _ in range(Config.NUM_BOIDS)]

    mouseDown = False

    active = True
    while active:
        dt = clock.tick()

        boidsByX, boidKeysByX, boidsByY, boidKeysByY = preprocessSearch(boids)

        for _, boid in enumerate(boids):
            xCandidates = candidatesInRange(boidsByX, boidKeysByX, boid.pos.x, Config.SCREEN_X, Config.VIEW_RADIUS)
            yCandidates = candidatesInRange(boidsByY, boidKeysByY, boid.pos.y, Config.SCREEN_Y, Config.VIEW_RADIUS)

            # Intersection of x candidate and y candidate boids, except the boid itself.
            # This is a RECTANGULAR region.
            rectNbors = set(xCandidates) & set(yCandidates) - set([boid])

            # Cap the maximum number of potential neighbours to Config.MAX_NBORS to avoid expensive operations with many boids.
            if len(rectNbors) > Config.MAX_NBORS:
                rectNbors = random.sample(rectNbors, Config.MAX_NBORS)

            nbors = [other for other in rectNbors if boid.can_see(other)]
            closeNbors = [other for other in nbors if boid.is_close(other)]

            # Rules affecting boids with neighbours

            if nbors:
                if Config.RULE1:  # Rule1: Boids fly to the centre of mass of neighbouring boids
                    nborCentreMass = sum(other.pos for other in nbors) / len(nbors)
                    boid.turn_to(nborCentreMass, magnitude=7)

                if Config.RULE2:  # Rule2: Boids try to keep a small distance away.
                    for other in closeNbors:
                        boid.turn_to(other.pos, magnitude=-5)

                if Config.RULE3:  # Rule3: Boids try to match angle with nearby boids.
                    # We cannot take the mean of the angles because of the discontinuity between pi and -pi.
                    # https://en.wikipedia.org/wiki/Mean_of_circular_quantities
                    y = sum(math.sin(other.vel.direction) for other in nbors)
                    x = sum(math.cos(other.vel.direction) for other in nbors)
                    meanAngle = math.atan2(y, x)
                    boid.turn_by(neg_mod(meanAngle - boid.vel.direction, math.pi) * dt / 5000)

            # Rules affecting all boids regardless of whether they have nbors

            if mouseDown:  # Run away from cursor on mouse down.
                x, y = pygame.mouse.get_pos()
                boid.turn_to(Vector2D(x, y), magnitude=-50)

            # Clamp velocity just in case it changes (it shouldn't)
            boid.vel = boid.vel.normalized() * Config.MAX_SPEED

        for boid in boids:
            # Move each boid according to the time passed
            boid.step(dt)

        draw_scene(window, boids)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                active = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouseDown = True
            if event.type == pygame.MOUSEBUTTONUP:
                mouseDown = False

    pygame.quit()


if __name__ == '__main__':
    main()
    # profile.run('main()', sort='tottime')
