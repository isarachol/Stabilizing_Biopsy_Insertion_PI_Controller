# Stabilizing Biopsy Insertion Needle using PI Controller

This is a final project for ME571 Medical Robotics at Boston University Spring 2025. The aim is to create an assistive device for biopsy operations that can help doctors (mainly medical trainees) stabilize a needle during an insertion operation. The program reads and calculates the orientations of the device and needle, displays the errors from the desired angle, and corrects it using a PI controller. It also reads and calculates the force value exerted onto the needle and displays that on the device.

![alt text](https://github.com/isarachol/Stabilizing_Biopsy_Insertion_PI_Controller/blob/master/final_proj.JPG)

## Problems/Limitations
- The orientation information is relative to the starting position and drifts over time
- Quick movements are not handled properly (dramatic drifts)
- Only corrects orientation in one axis (angle of device relative to the ground)
- The desired location is hard-coded

## Future Improvements
- Use a more sophisticated method to gather absolute orientation information such as incorporating magnetometers
- Use PID controller to handle quick movement which can lead to dramatic drifts
- Design more degree of freedom for the needle maneuver
- Add a User Interface or use a camera vision approach for online orientation determination
