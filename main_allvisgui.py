import pygame
import cv2
import time
import board
import psutil
import matplotlib.pyplot as plt
import numpy as np
from movement import *
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Robot Control System")
clock = pygame.time.Clock()

qr_detector = cv2.QRCodeDetector()
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

scanned_qr_codes = set()

left_motor = Motor(board.D17, board.D27)
right_motor = Motor(board.D10, board.D22)
movement = Movement(left_motor, right_motor)

i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50  # Hz
servo_1 = servo.Servo(pca.channels[12])
servo_2 = servo.Servo(pca.channels[13])
servo_3 = servo.Servo(pca.channels[14])
servo_back = servo.Servo(pca.channels[11])
servo_grip = servo.Servo(pca.channels[15])

servo_1.angle = 60
servo_2.angle = 180
servo_3.angle = 90
servo_back.angle = 1
servo_grip.angle = 90

step = 10

logo = pygame.image.load("logo.png")  # Logó betöltése
logo = pygame.transform.scale(logo, (100, 100))  # Méretezés

def save_qr_data(qr_data):
    if qr_data not in scanned_qr_codes:
        scanned_qr_codes.add(qr_data)
        with open("qr_codes.txt", "a") as file:
            file.write(f"{qr_data}\n")
        print(f"QR-code saved: {qr_data}")
    time.sleep(0.2)

def draw_cpu_usage():
    cpu_usage = psutil.cpu_percent()
    ram_usage = psutil.virtual_memory().percent
    
    font = pygame.font.Font(None, 24)
    cpu_text = font.render(f"CPU: {cpu_usage}%", True, (255, 255, 255))
    ram_text = font.render(f"RAM: {ram_usage}%", True, (255, 255, 255))
    
    screen.blit(cpu_text, (10, 550))
    screen.blit(ram_text, (10, 570))

def draw_grip_meter():
    center = (700, 500)
    radius = 50
    angle = np.radians(servo_grip.angle)
    pointer = (center[0] + int(radius * np.cos(angle - np.pi / 2)),
               center[1] + int(radius * np.sin(angle - np.pi / 2)))
    pygame.draw.circle(screen, (255, 255, 255), center, radius, 2)
    pygame.draw.line(screen, (255, 0, 0), center, pointer, 3)

def draw_arm_position():
    base_x, base_y = 400, 300
    scale = 2
    shoulder_x = base_x
    shoulder_y = base_y - servo_1.angle * scale
    elbow_x = shoulder_x + servo_2.angle * scale // 2
    elbow_y = shoulder_y - servo_2.angle * scale // 3
    wrist_x = elbow_x + servo_3.angle * scale // 2
    wrist_y = elbow_y - servo_3.angle * scale // 3
    
    pygame.draw.line(screen, (0, 255, 0), (base_x, base_y), (shoulder_x, shoulder_y), 5)
    pygame.draw.line(screen, (0, 255, 0), (shoulder_x, shoulder_y), (elbow_x, elbow_y), 5)
    pygame.draw.line(screen, (0, 255, 0), (elbow_x, elbow_y), (wrist_x, wrist_y), 5)
    pygame.draw.circle(screen, (255, 0, 0), (wrist_x, wrist_y), 5)

running = True
while running:
    screen.fill((50, 50, 50))
    screen.blit(logo, (10, 10))  # Logó megjelenítése
    font = pygame.font.Font(None, 30)
    text = font.render("Robot Control System", True, (255, 255, 255))
    screen.blit(text, (10, 120))  # Felirat a logó alatt

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a:
                print("left")
                movement.turnLeft()
            if event.key == pygame.K_d:
                print("right")
                movement.turnRight()
            if event.key == pygame.K_w:
                print("forward")
                movement.forward()
            if event.key == pygame.K_s:
                print("backward")
                movement.backward()
            if event.key == pygame.K_i:
                servo_1.angle = min(180, servo_1.angle + step)
            if event.key == pygame.K_k:
                servo_1.angle = max(60, servo_1.angle - step)
            if event.key == pygame.K_j:
                servo_2.angle = min(180, servo_2.angle + step)
            if event.key == pygame.K_l:
                servo_2.angle = max(0, servo_2.angle - step)
            if event.key == pygame.K_u:
                servo_3.angle = min(180, servo_3.angle + step)
            if event.key == pygame.K_o:
                servo_3.angle = max(0, servo_3.angle - step)
            if event.key == pygame.K_t:
                servo_grip.angle = min(180, servo_grip.angle + step)
            if event.key == pygame.K_g:
                servo_grip.angle = max(0, servo_grip.angle - step)
            if event.key == pygame.K_e:
                servo_back.angle = min(180, servo_back.angle + step)
            if event.key == pygame.K_q:
                servo_back.angle = max(0, servo_back.angle - step)
        if event.type == pygame.KEYUP:
            if event.key in (pygame.K_d, pygame.K_a, pygame.K_w, pygame.K_s):
                print("stopping")
                movement.stop()

    draw_cpu_usage()
    draw_grip_meter()
    draw_arm_position()
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
cap.release()
cv2.destroyAllWindows()
