import math

import pygame

import utils
from boid import Boid
from config import Config
from vector import Vector2D


def draw_scene(window, boids):
    window.fill((30, 30, 30))

    # Draw each boid.
    for boid in boids:
        if Config.BOID_SHAPE == 'DOT':
            pygame.draw.circle(window, boid.color, boid.pos, 1)
        elif Config.BOID_SHAPE == 'LINE':
            front, back = boid.get_line_points()
            pygame.draw.line(window, boid.color, front, back)
        else:
            points = boid.get_arrow_points()
            pygame.draw.polygon(window, boid.color, points)

    if Config.DRAW_FOV:
        pygame.draw.arc(window, (50, 50, 50), *boids[0].get_fov_arc_points(), 1)
        pygame.draw.lines(window, (50, 50, 50), False, boids[0].get_fov_line_points(), 1)

    # Flush display buffer
    pygame.display.flip()


def main():
    window = pygame.display.set_mode((Config.SCREEN_X, Config.SCREEN_Y))
    pygame.display.set_caption('Boids')
    clock = pygame.time.Clock()

    boids = [Boid() for _ in range(50)]
    mouseDown = False

    active = True
    while active:
        dt = clock.tick()

        for boid in boids:
            nbors = [other for other in boids if other is not boid and boid.can_see(other)]
            closeNbors = [other for other in nbors if boid.dist_squared(other) < 50**2]

            # Rules affecting boids with neighbours
            if nbors:
                if Config.RULE1:  # Rule1: Boids fly to the centre of mass of neighbouring boids
                    nborCentreMass = sum(other.pos for other in nbors) / len(nbors)
                    boid.turn_to(nborCentreMass, magnitude=5)

                if Config.RULE2:  # Rule2: Boids try to keep a small distance away.
                    for other in closeNbors:
                        boid.turn_to(other.pos, magnitude=-2)

                # We cannot take the mean of the angles because of the discontinuity between pi and -pi.
                # https://en.wikipedia.org/wiki/Mean_of_circular_quantities
                if Config.RULE3:  # Rule3: Boids try to match angle with nearby boids.
                    y = sum(math.sin(other.vel.direction) for other in nbors)
                    x = sum(math.cos(other.vel.direction) for other in nbors)
                    meanAngle = math.atan2(y, x)
                    boid.turn_by(utils.neg_mod(meanAngle - boid.vel.direction, math.pi) * dt / 10000)

            # Rules affecting all boids
            # Run away from cursor on mouse down.
            if mouseDown:
                x, y = pygame.mouse.get_pos()
                boid.turn_to(Vector2D(x, y), magnitude=50)

            # Clamp velocity just in case it changes (it shouldn't)
            boid.vel = boid.vel.normalized() * Config.MAX_SPEED

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
