#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serial
from objloader import OBJ  # Import the OBJ loader
import math

# ser = serial.Serial('/dev/tty.usbserial', 38400, timeout=1)
ser = serial.Serial('COM5', 9600, timeout=1)

ax = ay = az = 0.0
yaw_mode = False

# Load the OBJ model (global variable)
model = None

def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 700.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    global model
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
    
    # Load the OBJ model
    model = OBJ('./model/plane.obj')  # Adjust the path to your model
    model.generate()

def drawText(position, textString):
    font = pygame.font.SysFont("Courier", 18, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def draw_circle(x, y, radius, segments=100):
    """Draw a circle using OpenGL primitives."""
    glBegin(GL_LINE_LOOP)
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        dx = radius * math.cos(theta)
        dy = radius * math.sin(theta)
        glVertex2f(x + dx, y + dy)
    glEnd()

def draw_needle(x, y, angle, length):
    """Draw a needle for the meter."""
    glPushMatrix()
    glTranslatef(x, y, 0)
    glRotatef(angle, 0, 0, 1)
    glBegin(GL_LINES)
    glVertex2f(0, 0)
    glVertex2f(0, length)
    glEnd()
    glPopMatrix()

def draw_dashboard():
    # Switch to orthographic projection for the 2D dashboard
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(-1, 1, -1, 1)  # Orthographic view covering the whole window
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()

    # Draw the dashboard background
    glColor3f(0.1, 0.1, 0.1)  # Dark grey for the dashboard
    glBegin(GL_QUADS)
    glVertex2f(-1.0, -0.8)
    glVertex2f(1.0, -0.8)
    glVertex2f(1.0, -1.0)
    glVertex2f(-1.0, -1.0)
    glEnd()

    # Speed meter
    glColor3f(1.0, 1.0, 1.0)  # White for the meter outline
    draw_circle(-0.5, -0.9, 0.1)  # Draw the speed meter circle
    draw_needle(-0.5, -0.9, 45, 0.1)  # Example needle position for speed

    # Engine meter
    glColor3f(1.0, 1.0, 1.0)  # White for the meter outline
    draw_circle(0.5, -0.9, 0.1)  # Draw the engine meter circle
    draw_needle(0.5, -0.9, 90, 0.1)  # Example needle position for engine

    # Add labels for the meters
    drawText((-0.55, -0.95, 0), "Speed")
    drawText((0.45, -0.95, 0), "Engine")

    # Restore the previous projection and modelview matrices
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glPopMatrix()

def draw():
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Draw the sky background with a gradient
    glLoadIdentity()
    glDisable(GL_DEPTH_TEST)  # Disable depth testing for the background
    glBegin(GL_QUADS)

    # Top of the sky (light blue)
    glColor3f(0.4, 0.7, 0.9)  # Adjust these values for a lighter shade of blue
    glVertex3f(-1.0, 1.0, -1.0)
    glVertex3f(1.0, 1.0, -1.0)

    # Bottom of the sky (darker blue)
    glColor3f(0.0, 0.0, 0.5)  # Adjust these values for a darker blue
    glVertex3f(1.0, -0.3, -1.0)  # Adjusted y-coordinate to match ground height
    glVertex3f(-1.0, -0.3, -1.0)
    glEnd()

    # Draw the ground
    glBegin(GL_QUADS)
    glColor3f(0.55, 0.27, 0.07)  # Brown color for the ground
    glVertex3f(-1.0, -0.3, -1.0)  # Top-left corner (Adjusted y-coordinate to -0.3)
    glVertex3f(1.0, -0.3, -1.0)  # Top-right corner (Adjusted y-coordinate to -0.3)
    glVertex3f(1.0, -1.0, -1.0)  # Bottom-right corner
    glVertex3f(-1.0, -1.0, -1.0)  # Bottom-left corner
    glEnd()
    glEnable(GL_DEPTH_TEST)  # Re-enable depth testing

    # Set up the scene
    glLoadIdentity()
    glTranslatef(25, -100.0, -450.0)
    glRotatef(180, 0, 1, 0)

    osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax))

    if yaw_mode:
        osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(az))
    else:
        osd_line = osd_text

    drawText((-2, -2, 2), osd_line)

    # Apply IMU rotations
    if yaw_mode:
        glRotatef(az, 0.0, 1.0, 0.0)  # Yaw, rotate around y-axis
    glRotatef(ay, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis
    glRotatef(-1 * ax, 0.0, 0.0, 1.0)  # Roll, rotate around z-axis

    # Draw the loaded model
    if model:
        model.render()  # Call the render method of the OBJ loader

    # Draw the dashboard
    #draw_dashboard()

def read_data():
    global ax, ay, az
    ax = ay = az = 0.0
    line_done = 0

    # Request data by sending a dot
    ser.write(b".")  # Encode string to bytes
    line = ser.readline()
    angles = line.split(b", ")
    if len(angles) == 3:
        ax = float(angles[0])
        ay = float(angles[1])
        az = float(angles[2])
        line_done = 1

def main():
    global yaw_mode

    video_flags = OPENGL | DOUBLEBUF

    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(640, 480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    while True:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()  # Quit pygame properly
            break
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
            ser.write(b"z")
        read_data()
        draw()

        pygame.display.flip()
        frames += 1

    print("fps: %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))
    ser.close()

if __name__ == '__main__':
    main()
