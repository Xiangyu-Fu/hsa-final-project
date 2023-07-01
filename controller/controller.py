import pygame
import time


def communication_test() -> None:
    pygame.init()
    pygame.joystick.init()
    done = False

    while done != True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        joystick_count = pygame.joystick.get_count()

        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            axes = joystick.get_numaxes()

            print("===========================")

            time.sleep(0.1)

            for i in range(axes):
                axis = joystick.get_axis(i)
                print(axis)


if __name__ == "__main__":
    communication_test()
