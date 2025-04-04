import pygame
import cv2
import time
import board
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

left_motor = Motor( board.D17, board.D27 )
right_motor = Motor( board.D10, board.D22 )
movement = Movement( left_motor, right_motor )

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

def save_qr_data(qr_data):
    if qr_data not in scanned_qr_codes:
        scanned_qr_codes.add(qr_data)
        with open("qr_codes.txt", "a") as file:
            file.write(f"{qr_data}\n")
        print(f"QR-code saved: {qr_data}")
    
    time.sleep(0.2) 

running = True
while running:
    screen.fill((50, 50, 50))

    for event in pygame.event.get():
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
            if event.key == pygame.K_z:
                servo_1.angle = 60
                servo_2.angle = 180
                servo_3.angle = 90
                servo_back.angle = 1
                servo_grip.angle = 90
        if event.type == pygame.KEYUP:
            if event.key in ( pygame.K_d, pygame.K_a, pygame.K_w, pygame.K_s ):
                print("stopping")
                movement.stop()

        ret, frame = cap.read()
        if not ret:
            print("Nem sikerült beolvasni a kameraképet.")
            continue
        data, bbox, _ = qr_detector.detectAndDecode(frame)

        if bbox is not None and data:
            print(f"QR-kód észlelve: {data}")
            save_qr_data(data)
            for i in range(len(bbox)):
                points = bbox[i].astype(int)
                for j in range(len(points)):
                    cv2.line(frame, tuple(points[j]), tuple(points[(j+1) % len(points)]), (0, 255, 0), 2)
            cv2.putText(frame, data, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("QR Scanner - 1080p", frame)
        cv2.waitKey(1)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
#pca.deinit()
cap.release()
cv2.destroyAllWindows()
