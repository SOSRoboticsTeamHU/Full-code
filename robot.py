import pygame
import cv2
import time
import board
from movement import *
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Pygame inicializálása
pygame.init()
screen = pygame.display.set_mode((600, 400))
pygame.display.set_caption("Robot Control System")
clock = pygame.time.Clock()

# Kamera inicializálása
qr_detector = cv2.QRCodeDetector()
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FPS, 60)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Tárolt QR-kódok listája
scanned_qr_codes = set()

# Motorok inicializálása
left_motor = Motor(11, 13)
right_motor = Motor(19, 15)
movement = Movement(left_motor, right_motor)

# Szervók inicializálása
i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50  # Hz
servo_1 = servo.Servo(pca.channels[12])
servo_2 = servo.Servo(pca.channels[13])
servo_3 = servo.Servo(pca.channels[14])
servo_4 = servo.Servo(pca.channels[11]
servo_grip = servo.Servo(pca.channels[15])

# Alaphelyzet
servo_1.angle = 40
servo_2.angle = 60
servo_3.angle = 80
servo_grip.angle = 90

# Mozgási lépték
step = 2

def save_qr_data(qr_data):
    if qr_data not in scanned_qr_codes:
        scanned_qr_codes.add(qr_data)
        with open("qr_codes.txt", "a") as file:
            file.write(f"{qr_data}\n")
        print(f"QR-code saved: {qr_data}")

running = True
while running:
    ret, frame = cap.read()
    if ret:
        data, bbox, _ = qr_detector.detectAndDecode(frame)
        if bbox is not None and data:
            print(f"QR recognized: {data}")
            save_qr_data(data)
            for i in range(len(bbox)):
                points = bbox[i].astype(int)
                for j in range(len(points)):
                    cv2.line(frame, tuple(points[j]), tuple(points[(j+1) % len(points)]), (0, 255, 0), 2)
            cv2.putText(frame, data, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Camera View", frame)
    
    keys = pygame.key.get_pressed()
    
    # Robot mozgás (WASD)
    if keys[pygame.K_a]:
        movement.turnLeft()
    elif keys[pygame.K_d]:
        movement.turnRight()
    elif keys[pygame.K_w]:
        movement.forward()
    elif keys[pygame.K_s]:
        movement.backward()
    else:
        movement.stop()
    
    # Karvezérlés
    if keys[pygame.K_i]:
        servo_1.angle = min(180, servo_1.angle + step)
    if keys[pygame.K_k]:
        servo_1.angle = max(0, servo_1.angle - step)
    if keys[pygame.K_j]:
        servo_2.angle = min(180, servo_2.angle + step)
    if keys[pygame.K_l]:
        servo_2.angle = max(0, servo_2.angle - step)
    if keys[pygame.K_u]:
        servo_3.angle = min(180, servo_3.angle + step)
    if keys[pygame.K_o]:
        servo_3.angle = max(0, servo_3.angle - step)
    if keys[pygame.K_t]:
        servo_grip.angle = min(180, servo_grip.angle + step)
    if keys[pygame.K_g]:
        servo_grip.angle = max(0, servo_grip.angle - step)
    if keys[pygame.K_z]:
        servo_1.angle = 40
        servo_2.angle = 60
        servo_3.angle = 80
        servo_grip.angle = 90

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    clock.tick(60)
    time.sleep(0.05)

pygame.quit()
pca.deinit()
cap.release()
cv2.destroyAllWindows()
