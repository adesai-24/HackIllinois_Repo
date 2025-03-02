
# Sonic Shepherd 
_Combining AI with Animal Sciences to deliver the most comforting and efficient process to both livestock and farmers alike._

---

## Table of Contents

- [Project Description](#project-description)
- [Features](#features)
- [Challenges Encountered](#challenges-encountered)
- [Inspiration](#inspiration)
- [Future Implementations](#future-implementations)

---

## Project Description

This project is a robotic platform that leverages computer vision (CV) to interact with animals in a novel way. The robot uses a camera and CV algorithms to detect specific animals (like sheep) by identifying their unique features. Once a target animal is identified, the robot calculates its position relative to the camera frame, adjusts its orientation using a variety of sensors, and ultimately plays a specially chosen frequency. The goal is for the sound frequency to evoke a reaction in the target animal species, offering potential applications in animal behavior studies or livestock management.

---


## Features

- **Modular and Scalable Architecture:**  
  Designed with scalability in mind, our framework is easy to extend. Leveraging YOLO and a wealth of open-source resources, users can readily retrain or swap out models, add new sensors, or integrate additional functionalities with minimal effort for anybody from any background ranging from computer scientists to farmers who work on sight.

- **High-Performance Object Detection with YOLO:**  
  Unlike most teams that rely on conventional gradient-based methods, our approach uses a specially-trained YOLO model that we developed ourselves. This tailored model is optimized to detect target animals with higher accuracy.

- **Computer Vision Integration:**  
  Detects animals using bounding box techniques. The system calculates the bounding box dimensions and center to guide steering adjustments.

- **Proportional Steering Control:**  
  Implements a proportional loop (P control) to adjust the robot’s direction based on the detected object’s position relative to the camera’s center.

- **Targeted Frequency Emission:**  
  Designed to play a specific sound frequency intended to trigger a reaction in target animals (e.g., sheep).

- **Hardware Integration:**  
  Uses SunFounder’s Picarx platform and Picamera2, ensuring seamless interaction between sensors, motors, and the camera module.



---

## Challenges Encountered

Describe any challenges or obstacles you ran into during development and how you overcame them:

- **Faulty Hardware and Preparation:**  
  We encountered issues with the hardware setup. Instead of receiving a pre-imaged Raspberry Pi SD card, we had to image it ourselves, which delayed the project timeline.

- **Servo Motor Zeroing:**  
  There were significant challenges in zeroing the servo motors correctly, which affected the robot’s steering accuracy.

- **Communication Issues:**  
  Communication with the robot was problematic due to faulty WiFi hardware and an unprepared Raspberry Pi kit, making it difficult to reliably control and monitor the robot.

- **Low-Quality Camera:**  
  Although the camera was advertised with support for 720p at 30fps, its actual quality was very low. This compromised the performance of our image detection algorithms and made it harder to achieve reliable CV results.

- **Real-Time Processing:**  
  Balancing the processing load on the Raspberry Pi to ensure that both CV and control loops operate in real time.

---

## Inspiration


This project was inspired by the potential to use robotics and computer vision to interact with the animal world in innovative ways. The idea stemmed from:
- **Animal Behavior Research:**  
  Studying how animals respond to specific stimuli and frequencies.
- **Livestock Management Applications:**  
  Exploring non-invasive methods to herd or manage livestock using targeted auditory signals.
- **Robotics Integration:**  
  Combining real-time CV with traditional control systems (PID) to create a responsive, interactive platform.

---

## Future Implementations



- **Enhanced CV Algorithms:**  
  Integrate advanced machine learning models to improve the accuracy and reliability of animal detection under various conditions.

- **Adaptive Frequency Selection:**  
  Develop a system that dynamically adjusts the sound frequency based on the detected animal’s species or behavior patterns.

- **Obstacle Avoidance and Navigation:**  
  Incorporate additional sensors and algorithms to allow the robot to navigate more complex environments while maintaining focus on target animals.

- **Remote Monitoring & Control:**  
  Build a web or mobile interface to monitor the robot’s status and manually override controls if necessary.

- **Data Logging and Analysis:**  
  Implement logging of detection events and emitted frequencies to analyze animal responses over time.

---


