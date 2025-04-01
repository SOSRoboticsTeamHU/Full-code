import pygame
import cv2
import time
import board
import math
from movement import *
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

pygame.init()
screen = pygame.display.set_mode((600, 400))
pygame.display.set_caption("Robot Control System")
clock = pygame.time.Clock()

qr_detector = cv2.QRCodeDetector()
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
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

# Kezdő szervó pozíciók
servo_1.angle = 60
servo_2.angle = 180
servo_3.angle = 90
servo_back.angle = 1
servo_grip.angle = 90

step = 10
direction = "idle"

def save_qr_data(qr_data):
    if qr_data not in scanned_qr_codes:
        scanned_qr_codes.add(qr_data)
        with open("qr_codes.txt", "a") as file:
            file.write(f"{qr_data}\n")
        print(f"QR-code saved: {qr_data}")
    time.sleep(0.2)

def draw_arm():
    screen.fill((50, 50, 50))

    base_x, base_y = 300, 250

    angle1 = math.radians(servo_1.angle - 90)
    angle2 = math.radians(servo_2.angle - 90)
    angle3 = math.radians(servo_3.angle - 90)

    joint1_x = base_x + 50 * math.cos(angle1)
    joint1_y = base_y + 50 * math.sin(angle1)
    pygame.draw.line(screen, (0, 0, 255), (base_x, base_y), (joint1_x, joint1_y), 5)

    joint2_x = joint1_x + 50 * math.cos(angle1 + angle2)
    joint2_y = joint1_y + 50 * math.sin(angle1 + angle2)
    pygame.draw.line(screen, (0, 255, 0), (joint1_x, joint1_y), (joint2_x, joint2_y), 5)

    end_x = joint2_x + 30 * math.cos(angle1 + angle2 + angle3)
    end_y = joint2_y + 30 * math.sin(angle1 + angle2 + angle3)
    pygame.draw.line(screen, (255, 0, 0), (joint2_x, joint2_y), (end_x, end_y), 5)

    gripper_color = (0, 255, 0) if servo_grip.angle > 90 else (255, 0, 0)
    pygame.draw.circle(screen, gripper_color, (int(end_x), int(end_y)), 10)

    font = pygame.font.Font(None, 36)
    text = font.render(f"Robot Mozgás: {direction}", True, (255, 255, 255))
    screen.blit(text, (10, 10))

    pygame.display.flip()

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a:
                movement.turnLeft()
                direction = "left"
            if event.key == pygame.K_d:
                movement.turnRight()
                direction = "right"
            if event.key == pygame.K_w:
                movement.forward()
                direction = "forward"
            if event.key == pygame.K_s:
                movement.backward()
                direction = "backward"
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
            if event.key == pygame.K_z:
                servo_1.angle = 60
                servo_2.angle = 180
                servo_3.angle = 90
                servo_back.angle = 1
                servo_grip.angle = 90

        if event.type == pygame.KEYUP:
            if event.key in (pygame.K_d, pygame.K_a, pygame.K_w, pygame.K_s):
                movement.stop()
                direction = "idle"


    ret, frame = cap.read()
    if ret:
        data, bbox, _ = qr_detector.detectAndDecode(frame)
        if bbox is not None and data:
            save_qr_data(data)
        cv2.imshow("QR Scanner", frame)
        cv2.waitKey(1)

    draw_arm()
    clock.tick(60)

pygame.quit()
cap.release()
cv2.destroyAllWindows()
