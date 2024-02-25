import serial
import time
import cv2
import mediapipe as mp
import numpy as np

NUMBER_OF_FINGERS = 5
VIDEO_CHANNEL = 0
FINGER_RESPONSE_TIME = 0.1

class handDetector():
    def __init__(self, mode=False, maxHands=1, detectionCon=0.90, trackCon=0.9):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, 1, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=True):

        lmlist = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy, cz = int(lm.x * w), int(lm.y * h)#, int(lm.z * w)
                #lmlist.append([id, cx, cy, cz])
                lmlist.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 7, (255, 0, 255), cv2.FILLED)
        return lmlist


def set_target(serial_port, channel, target):
    target_bytes = bytearray([0x84, channel, target & 0x7F, (target >> 7) & 0x7F])
    serial_port.write(target_bytes)


def get_position(serial_port, channel):
    serial_port.write(bytearray([0x90, channel]))
    response = serial_port.read(2)
    position = response[0] + 256 * response[1]
    return position


def pointsToAngle(start_point, middle_point, end_point):
    start_point = np.array(start_point)
    middle_point = np.array(middle_point)
    end_point = np.array(end_point)

    vector1 = start_point - middle_point
    vector2 = end_point - middle_point

    dot_product = np.dot(vector1, vector2)

    norm_vector1 = np.linalg.norm(vector1)
    norm_vector2 = np.linalg.norm(vector2)

    cos_theta = dot_product / (norm_vector1 * norm_vector2)

    angle = np.arccos(cos_theta)
    angle_degrees = np.degrees(angle)

    return angle_degrees


def findAngles(landMarkList):
    # angle1 = pointsToAngle((landMarkList[17][1], landMarkList[17][2], landMarkList[17][3]),
    #                        (landMarkList[18][1], landMarkList[18][2], landMarkList[18][3]),
    #                        (landMarkList[19][1], landMarkList[19][2], landMarkList[19][3]))  # little finger
    # angle2 = pointsToAngle((landMarkList[13][1], landMarkList[13][2], landMarkList[13][3]),
    #                        (landMarkList[14][1], landMarkList[14][2], landMarkList[14][3]),
    #                        (landMarkList[15][1], landMarkList[15][2], landMarkList[15][3]))  # ring finger
    # angle3 = pointsToAngle((landMarkList[9][1], landMarkList[9][2], landMarkList[9][3]),
    #                        (landMarkList[10][1], landMarkList[10][2], landMarkList[10][3]),
    #                        (landMarkList[11][1], landMarkList[11][2], landMarkList[11][3]))  # middle finger
    # angle4 = pointsToAngle((landMarkList[5][1], landMarkList[5][2], landMarkList[5][3]),
    #                        (landMarkList[6][1], landMarkList[6][2], landMarkList[6][3]),
    #                        (landMarkList[7][1], landMarkList[7][2], landMarkList[7][3]))  # index finger
    # angle5 = pointsToAngle((landMarkList[2][1], landMarkList[2][2], landMarkList[2][3]),
    #                        (landMarkList[3][1], landMarkList[3][2], landMarkList[3][3]),
    #                        (landMarkList[4][1], landMarkList[4][2], landMarkList[4][3]))  # thumb
    angle1 = pointsToAngle((landMarkList[17][1], landMarkList[17][2]),
                           (landMarkList[18][1], landMarkList[18][2]),
                           (landMarkList[19][1], landMarkList[19][2]))  # little finger
    angle2 = pointsToAngle((landMarkList[13][1], landMarkList[13][2]),
                           (landMarkList[14][1], landMarkList[14][2]),
                           (landMarkList[15][1], landMarkList[15][2]))  # ring finger
    angle3 = pointsToAngle((landMarkList[9][1], landMarkList[9][2]),
                           (landMarkList[10][1], landMarkList[10][2]),
                           (landMarkList[11][1], landMarkList[11][2]))  # middle finger
    angle4 = pointsToAngle((landMarkList[5][1], landMarkList[5][2]),
                           (landMarkList[6][1], landMarkList[6][2]),
                           (landMarkList[7][1], landMarkList[7][2]))  # index finger
    angle5 = pointsToAngle((landMarkList[2][1], landMarkList[2][2]),
                           (landMarkList[3][1], landMarkList[3][2]),
                           (landMarkList[4][1], landMarkList[4][2]))  # thumb

    list_of_angles = {angle1, angle2, angle3, angle4, angle5}
    return list_of_angles


def scale_value(original_value, original_min=0, original_max=180, new_min=2000, new_max=8000):
    return ((original_value - original_min) / (original_max - original_min)) * (new_max - new_min) + new_min


def main():
    with serial.Serial('/dev/tty.usbmodem004148121', 9600) as maestro_serial:
        cap = cv2.VideoCapture(VIDEO_CHANNEL)
        detector = handDetector()
        while True:
            success, img = cap.read()
            img = detector.findHands(img)
            lmlist = detector.findPosition(img)
            if len(lmlist) != 0:

                angle_list = findAngles(lmlist)
                angle_list = list(angle_list)
                for finger_number in range(NUMBER_OF_FINGERS):
                    try:
                        setvaluenew = int(scale_value(angle_list[finger_number]))
                        set_target(maestro_serial, finger_number, setvaluenew)
                    except:
                        pass
                time.sleep(FINGER_RESPONSE_TIME)


if __name__ == '__main__':
    main()
