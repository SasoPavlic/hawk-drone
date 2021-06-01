import cv2
import numpy as np
import platform

# To finish recording after some time
_MAX_FRAMES = 5000

# Create a VideoCapture object
cap = cv2.VideoCapture(0)

# Check if camera opened successfully
if (cap.isOpened() == False):
    print("Unable to read camera feed")

# Default resolutions of the frame are obtained.The default resolutions are system dependent.
# We convert the resolutions from float to integer.
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
counter = 0

out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 30.0, (frame_width, frame_height))

while(True):
    counter += 1

    ret, frame = cap.read()
    frame = cv2.flip( frame, -1 )

    if ret == True:

        # Write the frame into the file 'output.avi'
        out.write(frame)

        if platform.uname()[4] == "x86_64":
            cv2.imshow("Frame", frame)

        key=cv2.waitKey(1) & 0xFF

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            print("Key Q pressed...")
            break
        elif counter == _MAX_FRAMES:
            break

    # Break the loop
    else:
        break

# When everything done, release the video capture and video write objects
cap.release()
out.release()

# Closes all the frames
cv2.destroyAllWindows()

