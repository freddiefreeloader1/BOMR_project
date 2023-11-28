from ultralytics import YOLO
import cv2
import math

cap = cv2.VideoCapture(0)

classNames = ['Thymio']

model = YOLO("./runs/detect/train/weights/best.pt")

while True:
    success, img = cap.read()

    try:
        results = model(img, stream=True)

        # coordinates
        for r in results:
            boxes = r.boxes
            # print('Results:')
            # print(r.boxes)
            # print('------')

            for box in boxes:
                _, _, w, h = box.xywh[0]
                w, h = int(w), int(h)

                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                center = (x1 + (w // 2), y1 + (h // 2))

                cv2.circle(img, center, 7, (0, 255, 255), -1)
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                confidence = math.ceil((box.conf[0] * 100)) / 100
                print("Confidence --->",confidence)

                cls = int(box.cls[0])
                print("Class name -->", classNames[cls])

                # object details
                org = [x1, y1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 255, 0)
                thickness = 2

                cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)
    except Exception as e:
        print("Error:", e)
        continue

    cv2.imshow('Frame', img)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
