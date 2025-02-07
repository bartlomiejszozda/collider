import pygame
import sys

# Initialize pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check for available joysticks
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    pygame.quit()
    sys.exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Print the name of the joystick
print("Joystick initialized:", joystick.get_name())

# Run the game loop to check for events
try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            # Axis input (e.g., for a gamepad stick)
            if event.type == pygame.JOYAXISMOTION:
                axis = event.axis
                value = joystick.get_axis(axis)
                print(f"Axis {axis} moved with value: {value}")

            # Button input (e.g., for buttons on a gamepad)
            if event.type == pygame.JOYBUTTONDOWN:
                button = event.button
                print(f"Button {button} pressed")

            if event.type == pygame.JOYBUTTONUP:
                button = event.button
                print(f"Button {button} released")

except KeyboardInterrupt:
    print("Exiting the program.")
    pygame.quit()
    sys.exit()

