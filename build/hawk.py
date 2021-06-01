# developed by: Sašo Pavlič, Peter Vrbančič

# Import modules
from imutils.video import VideoStream
from imutils.video import FPS
from multiprocessing import Process
from multiprocessing import Queue
from subprocess import Popen, PIPE
import cv2
import math
import sys
import os
import argparse
import time
import platform
from queue import Queue as surfaceQueue
import numpy as np

# Directional commands
_HAWK_MODE = False
_TAKEOFF = False
_LAND = False
_MOVE_DETECTION = "HOLD"

# Limit file writes
file_writer_counter = 0

# Set max frame width and height
frame_width = 640  # video.frame.shape[1]
frame_height = 480  # video.frame.shape[0]

# Prepare VideoWriter, we will write 30 FPS into file
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(
    *'XVID'), 30.0, (frame_width, frame_height))

# initialize the input queue (frames), output queue (detections),
# and the list of actual detections returned by the child process
inputQueue = Queue(maxsize=1)
outputQueue = Queue(maxsize=1)

# Surface area Queue, in some cases we can increase maxsize to get
# AVG surface area of multiple frames, in this example we only used 1
# mainly because of slow device
avg_surfacesQueue = surfaceQueue(maxsize=1)

# referecne point, it is set up during first few frames
ref_avg_area = 0.0

# Writing detectional commands to text file. Text file is used by C++ script to control MAVSDK commands


def write_detection(priority, command):
    global file_writer_counter
    if file_writer_counter == 100 or priority:
        detection_output = open("pythonToCppCommand.txt", "w")
        detection_output.write(str(command))
        detection_output.close()
        file_writer_counter = 0
    else:
        file_writer_counter = file_writer_counter + 1

# Relative of circle cordinates calculate next move for drone


def get_direction(circle_cord):
    global _MOVE_DETECTION
    ax = 125
    ay = 375
    bx = 525
    by = 375
    cx = 525
    cy = 125
    dx = 125
    dy = 125

    x = circle_cord[0]
    y = circle_cord[1]

    bax = bx - ax
    bay = by - ay
    dax = dx - ax
    day = dy - ay

    # PRIORITY 1
    # If circle_cord is on LEFT/RIGHT CLIMB/DECLINE side of frame we will check following conditions

    if ((x - ax) * bax + (y - ay) * bay < 0.0):
        if x < ax and y > ay:
            _MOVE_DETECTION = "LEFT-DECLINE"
            return
        elif x < dx and y < dy:
            _MOVE_DETECTION = "LEFT-CLIMB"
            return
        else:
            _MOVE_DETECTION = "LEFT"
            return
    if ((x - bx) * bax + (y - by) * bay > 0.0):
        if x > bx and y > by:
            _MOVE_DETECTION = "RIGHT-DECLINE"
            return
        elif x > cx and y < cy:
            _MOVE_DETECTION = "RIGHT-CLIMB"
            return
        else:
            _MOVE_DETECTION = "RIGHT"
            return
    if ((x - ax) * dax + (y - ay) * day < 0.0):
        _MOVE_DETECTION = "DECLINE"
        return
    if ((x - dx) * dax + (y - dy) * day > 0.0):
        _MOVE_DETECTION = "CLIMB"
        return

    # PRIORITY 2
    # IF circle_cord was not on edge of frame lets check distance to drone to get FORWARD/REVERSE
    global ref_avg_area

    sum_area = 0.0

    # If we are using device with lover computing power we are calculating move based on average
    # depends of maxsize of avg_surfacesQueue
    for i in avg_surfacesQueue.queue:
        sum_area = sum_area + i

    avg_area = sum_area/avg_surfacesQueue.qsize()

    if ref_avg_area == 0.0:
        ref_avg_area = avg_area
        avg_surfacesQueue.queue.clear()

    else:

        if (avg_area > (ref_avg_area - (ref_avg_area * 0.20))) and (avg_area < (ref_avg_area + (ref_avg_area * 0.20))):
            _MOVE_DETECTION = "HOLD"

        elif avg_area >= (ref_avg_area + (ref_avg_area * 0.20)):
            _MOVE_DETECTION = "REVERSE"

            if avg_area > (ref_avg_area + (ref_avg_area * 0.70)):
                print("HAWK_MODE: ON")
                _MOVE_DETECTION = "HAWK_MODE"

        elif avg_area <= (ref_avg_area - (ref_avg_area * 0.20)):
            _MOVE_DETECTION = "FORWARD"

        avg_surfacesQueue.queue.clear()

# based on content of frame we will get all detected objects
# available object are located on this link: https://tech.amikelive.com/node-718/what-object-categories-labels-are-in-coco-dataset/
def classify_frame(tensorFlowNet, inputQueue, outputQueue):
    # keep looping
    while True:
        # check to see if there is a frame in our input queue
        if not inputQueue.empty():
            # grab the frame from the input queue, resize it, and
            # construct a blob from it
            frame = inputQueue.get()
            frame = cv2.resize(frame, (300, 300))
            blob = cv2.dnn.blobFromImage(
                frame, size=(300, 300), swapRB=True, crop=False)

            # set the blob as input to our deep learning object
            # detector and obtain the detections
            tensorFlowNet.setInput(blob)
            detections = tensorFlowNet.forward()

            # write the detections to the output queue
            outputQueue.put(detections)


# main fucntion which controls entire process
def main():

    # initialize the bounding box coordinates of the object we are going to track
    detections = None
    counter = 0

    global _TAKEOFF
    global _LAND
    global _MOVE_DETECTION

    _TAKEOFF = True
    _LAND = False
    _MOVE_DETECTION = "HOLD"

    # Tell to drone to takeoff and wait few seconds
    write_detection(True, "TAKEOFF")
    time.sleep(10)

    if not args.get("video", False):
        video = VideoStream(src=0).start()
        time.sleep(1.0)
        fps = FPS().start()
    else:
        video = cv2.VideoCapture(args["video"])
        # Exit if video not opened
        if not video.isOpened():
            print("Could not open video")
            sys.exit()

        # Read first frame
        ok, frame = video.read()
        if not ok:
            print("Cannot read video file")
            sys.exit()

        frame = video.read()
        frame = frame[1] if args.get("video", False) else frame
        frame = cv2.resize(frame, (640, 480))

        # start the FPS throughput estimator as well
        fps = FPS().start()
        print(f"frame dimensions:{frame.shape}")

    print(f"Dimensions for output file:{frame_width}x{frame_height}")
    print("Starting loop...")
    # loop over frames from the video stream
    start_time = time.time()

    # start getting video stream and detect objects on frame by frame
    while True:
        counter += 1
        # grab the current frame, then handle if we are using a
        # VideoStream or VideoCapture object
        frame = video.read()
        # frame = cv2.flip(frame, 1)
        frame = frame[1] if args.get("video", False) else frame

        # check to see if we have reached the end of the stream
        if frame is None:
            break

        # resize the frame (so we can process it faster) and grab the
        # frame dimensions
        frame = cv2.resize(frame, (640, 480))
        (H, W) = frame.shape[:2]

        # if the input queue *is* empty, give the current frame to
        # classify
        if inputQueue.empty():
            inputQueue.put(frame)

        # if the output queue *is not* empty, grab the detections
        if not outputQueue.empty():
            detections = outputQueue.get()

        # check to see if our detectios are not None (and if so, we'll
        # draw the detections on the frame)
        if detections is not None:
            # loop over the detections
            for i in np.arange(0, detections.shape[2]):
                # extract the confidence (i.e., probability) associated
                # with the prediction
                confidence = detections[0, 0, i, 2]

                # filter out weak detections by ensuring the `confidence`
                # is greater than the minimum confidence
                if confidence > args["confidence"]:

                    # otherwise, extract the index of the class label from
                    # the `detections`, then compute the (x, y)-coordinates
                    # of the bounding box for the object
                    idx = int(detections[0, 0, i, 1])
                    dims = np.array([W, H, W, H])
                    box = detections[0, 0, i, 3:7] * dims
                    (startX, startY, endX, endY) = box.astype("int")

                    if idx == 1:  # 1 = Person:
                        bbox = (startX, startY, endX, endY)

                        # draw the prediction on the frame
                        label = "{}: {:.2f}%".format(
                            CLASSES[idx], confidence * 100)

                        # get rectangle cordinates
                        ax = bbox[0]
                        ay = bbox[1]

                        bx = bbox[2]
                        by = bbox[1]

                        cx = bbox[2]
                        cy = bbox[3]

                        dx = bbox[0]
                        dy = bbox[3]

                        x = [ax, bx, cx, dx]
                        y = [ay, by, cy, dy]

                        # calculate surface area of rectangle
                        w = math.sqrt((bx-ax)*(bx-ax) + (by-ay)*(by-ay))
                        h = math.sqrt((cx-bx)*(cx-bx) + (cy-by)*(cy-by))

                        surface = int(w*h)
                        print(f"SURFACE: {surface}")

                        # add current surface to queue (if we are using slower device we dont want to move for each frame)
                        avg_surfacesQueue.put(surface)

                        # calculate center of rectangle
                        circle_cord = ((startX+endX)//2, (startY+endY)//2)
                        
                        # calculate direction of drone
                        get_direction(circle_cord)

                        avg_surfacesQueue.queue.clear()

                        print(f"Move: {_MOVE_DETECTION}")

                        cv2.rectangle(frame, (startX, startY),
                                      (endX, endY), COLORS[idx], 2)
                        cv2.circle(frame, circle_cord, radius=10,
                                   color=COLORS[idx], thickness=-1)

                        y = startY - 15 if startY - 15 > 15 else startY + 15
                        cv2.putText(frame, label, (startX, y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

                        print(f"Object detected: {label}")
                        print(f"Center X,Y: {circle_cord}")

                        # breaking detection loop to display only 1 object on frame (most confident one)
                        # if we don't do this we will sometimes get 2 objects which can disturb decision to which direction we are going.
                        break

        # update the FPS counter
        fps.update()
        fps.stop()
        # initialize the set of information we'll be displaying on
        # the frame
        info = [
            ("FPS", "{:.2f}".format(fps.fps())),
            ("MOVE", _MOVE_DETECTION)
        ]

        # loop over the info tuples and draw them on our frame
        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # horizontal and vertical line on frame, representing center of frame
        cv2.line(frame, pt1=(frame_width//2, 0), pt2=(frame_width //
                                                      2, frame_height), color=(0, 0, 255), thickness=1)
        cv2.line(frame, pt1=(0, frame_height//2), pt2=(frame_width,
                                                       frame_height//2), color=(0, 0, 255), thickness=1)

        # write calculated detection to file
        write_detection(False, _MOVE_DETECTION)
        out.write(frame)

        # if we have headless device we don't display frame
        if platform.uname()[4] == "x86_64":
            cv2.imshow("Frame", frame)

        key = cv2.waitKey(1) & 0xFF

        # if the `q` key was pressed or counter is reached, break from the loop
        if key == ord("q"):
            print("Key Q pressed...")
            break
        elif counter == 5000:
            break

    # last action when ending script is to land
    _TAKEOFF = False
    _LAND = True
    write_detection(True, "LAND")
    print("--- %s seconds ---" % (time.time() - start_time))

    # if we are using a webcam, release the pointer
    if not args.get("video", False):
        video.stop()
        out.release()

    # otherwise, release the file pointer
    else:
        video.release()
        out.release()

    # close all windows
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # Construct the argument parser and parse the argumetns
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", type=str,
                    help="path to input video fileeeee")
    ap.add_argument("-p", "--prototxt", default="model.pbtxt",
                    help="path to Tensor 'deploy' prototxt file")
    ap.add_argument("-m", "--model", default="model.pb",
                    help="path to Tensor pre-trained model")
    ap.add_argument("-c", "--confidence", type=float, default=0.3,
                    help="minimum probability to filter weak detections")
    args = vars(ap.parse_args())

    # initialize the list of class labels MobileNet SSD was trained to
    # detect, then generate a set of bounding box colors for each class
    CLASSES = {1: "person",
               43: "tennis racket",
               44: "bottle"}

    COLORS = np.random.uniform(0, 255, size=(255, 3))

    # check if files exists in folder
    if not (os.path.isfile(args["model"]) and os.path.isfile(args["prototxt"])):
        errorMsg = '''
        Could not find TensorFlow model in current directory.
        Please ensure NAME.pb and NAME.pbtxt are in the current directory
        '''

        print(errorMsg)
        sys.exit()

    #START C++ script to control MAVSDK commands
    Popen("gnome-terminal -x " + "./takeoff_and_land udp://14540",
          stdout=PIPE, stderr=PIPE, stdin=PIPE, shell=True)

    # load our serialized model from disk
    print("[INFO] loading model...")
    net = cv2.dnn.readNetFromTensorflow(args["model"], args["prototxt"])

    # start detection in seperated process
    print("[INFO] starting process...")
    p_classify = Process(target=classify_frame,
                         args=(net, inputQueue, outputQueue))
    p_classify.daemon = True
    p_classify.start()

    # start main functions
    main()

    # output PIDs in case we need to kill them manually
    print(f"main pid: {os.getpid()}")
    print(f"p_classify pid: {p_classify.pid}")

    print(f"[INFO] Emptying input and output queue")
    while not inputQueue.empty():
        print("waiting...")
        time.sleep(1)

    p_classify.terminate()
    p_classify.join(timeout=1.0)
    p_classify.kill()

    print("[INFO] Done")
    sys.exit()
