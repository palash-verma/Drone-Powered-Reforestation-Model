import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np

def connect_drone():
    vehicle = connect('127.0.0.1:14550', wait_ready=True)
    return vehicle

def arm_and_takeoff(vehicle, altitude):
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(altitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def recognize_terrain(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Use Canny edge detection
    edges = cv2.Canny(gray, 100, 200)
    
    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        if cv2.contourArea(contour) > 1000:  # Adjust the area threshold as needed
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
    return frame, contours

def main():
    vehicle = connect_drone()
    arm_and_takeoff(vehicle, 10)  # Takeoff to 10 meters altitude

    cap = cv2.VideoCapture(0)  # Assume the drone's camera is available at /dev/video0 or similar

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        processed_frame, contours = recognize_terrain(frame)

        # Display the resulting frame
        cv2.imshow('Frame', processed_frame)

        # Drop seeds and irrigate based on contours
        for contour in contours:
            if cv2.contourArea(contour) > 2000:  # Example threshold for decision making
                drop_seeds(vehicle.location.global_frame)
                irrigate(vehicle.location.global_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    vehicle.close()

def drop_seeds(location):
    print("Dropping seeds at:", location)
    # Code to drop seeds

def irrigate(location):
    print("Irrigating at:", location)
    # Code to irrigate

if __name__ == '__main__':
    main()
