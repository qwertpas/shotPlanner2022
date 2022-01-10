
import pygame

window = pygame.display.set_mode((500,500))
red = (200,0,0)

circleX = 100
circleY = 100
radius = 10

active = True

while active:
   for event in pygame.event.get():
      if event.type == pygame.QUIT:
         active = False

   pygame.draw.circle(window,red,(circleX,circleY),radius) # DRAW CIRCLE

   pygame.display.update()