import cv2
import time

# Initialize the video capture object
cap = cv2.VideoCapture(0)  # 0 is the default camera

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

# Initialize timer
start_time = time.time()
photo_interval = 1  # Interval in seconds

photo_count = 0

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        # Write the frame to the output file
        out.write(frame)

        # Display the frame
        cv2.imshow('frame', frame)

        # Check if the interval time has passed
        current_time = time.time()
        if current_time - start_time >= photo_interval:
            # Save the frame as an image
            photo_filename = f'photo_{photo_count:04d}.jpg'
            cv2.imwrite(photo_filename, frame)
            photo_count += 1
            start_time = current_time  # Reset the timer

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()
