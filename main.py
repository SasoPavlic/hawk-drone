# developed by: Sašo Pavlič

# Import modules
from imutils.video import VideoStream
from imutils.video import FPS
from multiprocessing import Process
from multiprocessing import Queue
import imutils
import cv2
import sys
import os
import argparse
import time
import platform
import numpy as np


_FINISH = False



# Construct the argument parser and parse the argumetns
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
                help="path to input video fileeeee")
ap.add_argument("-p", "--prototxt", default="MobileNetSSD_deploy.prototxt.txt",
                help="path to Caffe 'deploy' prototxt file")
ap.add_argument("-m", "--model", default="MobileNetSSD_deploy.caffemodel",
                help="path to Caffe pre-trained model")
ap.add_argument("-c", "--confidence", type=float, default=0.20,
                help="minimum probability to filter weak detections")
args = vars(ap.parse_args())

# initialize the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
        "sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))


if not (os.path.isfile('MobileNetSSD_deploy.caffemodel') and os.path.isfile('MobileNetSSD_deploy.prototxt.txt')):
    errorMsg = '''
    Could not find GOTURN model in current directory.
    Please ensure goturn.caffemodel and goturn.prototxt are in the current directory
    '''

    print(errorMsg)
    sys.exit()

frame_width = 640  # video.frame.shape[1]
frame_height = 480  # video.frame.shape[0]

out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (frame_width, frame_height))

# load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])

# initialize the input queue (frames), output queue (detections),
# and the list of actual detections returned by the child process
inputQueue = Queue(maxsize=1)
outputQueue = Queue(maxsize=1)
writerQueue = Queue(maxsize=1)


def write_to_file(frame, frame_stack):  
    while True:
        if _FINISH:
            break
        if not frame_stack.empty():
            out.write(frame_stack.get())

def classify_frame(net, inputQueue, outputQueue):
    # keep looping
    while True:
        if _FINISH:
            break
        # check to see if there is a frame in our input queue
        if not inputQueue.empty():
            # grab the frame from the input queue, resize it, and
            # construct a blob from it
            frame = inputQueue.get()
            frame = cv2.resize(frame, (300, 300))            
            blob = cv2.dnn.blobFromImage(frame, 0.007843,
                                         (300, 300), 127.5)

            # set the blob as input to our deep learning object
            # detector and obtain the detections
            net.setInput(blob)
            detections = net.forward()

            # write the detections to the output queue
            outputQueue.put(detections)

def get_direction(circle_cord):
    ax = 100
    ay = 400
    bx = 550
    by = 400
    dx = 100
    dy = 100

    x = circle_cord[0]
    y = circle_cord[1]

    bax = bx - ax
    bay = by - ay
    dax = dx - ax
    day = dy - ay

    if ((x - ax) * bax + (y - ay) * bay < 0.0):
        if x < 100 and y > 400:
            return "LEFT-DOWN"
        elif x < 100 and y < 100:
            return "LEFT-UP"
        else:
            return "LEFT"
    if ((x - bx) * bax + (y - by) * bay > 0.0):
        if x > 550 and y > 400:
            return "RIGHT-DOWN"
        elif x > 550 and y < 100:
            return "RIGHT-UP"
        else:
            return "RIGHT"
    if ((x - ax) * dax + (y - ay) * day < 0.0): return "DOWN"
    if ((x - dx) * dax + (y - dy) * day > 0.0): return "UP"

    return "HOLD"

def start():
      
    # initialize the bounding box coordinates of the object we are going
    # to track
    initBB = None
    frame_num = 0
    detections = None


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
        
        #start the FPS throughput estimator as well
        fps = FPS().start()
        print(f"frame dimensions:{frame.shape}")

    print(f"Dimensions for output file:{frame_width}x{frame_height}")
    print("Starting loop...")
    # loop over frames from the video stream
    counter = 0
    start_time = time.time()
    object_detected = False
    move_direction = "HOLD"        



    while True:
        counter += 1
        # grab the current frame, then handle if we are using a
        # VideoStream or VideoCapture object
        frame = video.read()
        frame_num += 1
        #frame = cv2.flip(frame, 1)
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
        
        if object_detected != True:
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
                    if confidence < args["confidence"]:
                        continue

                    # otherwise, extract the index of the class label from
                    # the `detections`, then compute the (x, y)-coordinates
                    # of the bounding box for the object
                    idx = int(detections[0, 0, i, 1])
                    dims = np.array([W, H, W, H])
                    box = detections[0, 0, i, 3:7] * dims
                    (startX, startY, endX, endY) = box.astype("int")
                    if CLASSES[idx] == "person":                    
                        bbox = (startX, startY, endX, endY)
                        
                        # draw the prediction on the frame
                        label = "{}: {:.2f}%".format(CLASSES[idx],confidence * 100)                   

                        circle_cord = ((startX+endX)//2,(startY+endY)//2)
                        move_direction = get_direction(circle_cord)
                        print(f"Move: {move_direction}")

                        cv2.rectangle(frame, (startX, startY), (endX, endY),COLORS[idx], 2)
                        cv2.circle(frame, circle_cord, radius=10, color=COLORS[idx], thickness=-1)      
                        
                        y = startY - 15 if startY - 15 > 15 else startY + 15
                        cv2.putText(frame, label, (startX, y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)                    
                                
                        print(f"Object detected: {label}")
                        print(f"Center X,Y: {circle_cord}")
                        
        # update the FPS counter
        fps.update()
        fps.stop()
        # initialize the set of information we'll be displaying on
        # the frame
        info = [            
            ("FPS", "{:.2f}".format(fps.fps())),
            ("MOVE", move_direction)
        ]

        # loop over the info tuples and draw them on our frame
        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.line(frame, pt1=(frame_width//2,0), pt2=(frame_width//2,frame_height), color=(0,0,255), thickness=1)
        cv2.line(frame, pt1=(0,frame_height//2), pt2=(frame_width,frame_height//2), color=(0,0,255), thickness=1)
        writerQueue.put(frame)        
        
        if platform.uname()[4] == "x86_64":
            cv2.imshow("Frame", frame)

        key = cv2.waitKey(1) & 0xFF

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            print("Key Q pressed...")        
            break
        elif counter == 500:
            break

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
    print("[INFO] starting process...")
    
    frame = ""
    p1 = Process(target=write_to_file, args=(frame,writerQueue))
    p1.daemon = True
    p1.start() 

    p2 = Process(target=classify_frame, args=(net, inputQueue,outputQueue))
    p2.daemon = True
    p2.start()

    start()

   

    print(f"main pid: {os.getpid()}")
    print(f"p1 pid: {p1.pid}")
    print(f"p2 pid: {p2.pid}")

    
    p1.terminate()
    p2.terminate()
    p1.join(timeout=1.0)
    p2.join(timeout=1.0)

    net = None
    #inputQueue = None
    #outputQueue = None
    #writerQueue = None
    #https://stackoverflow.com/questions/32053618/how-to-to-terminate-process-using-pythons-multiprocessing
    #https://cuyu.github.io/python/2016/08/15/Terminate-multiprocess-in-Python-correctly-and-gracefully

    print("[INFO] Done")
    sys.exit()
