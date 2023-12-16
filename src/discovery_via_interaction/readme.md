# Overview:

in discovery_via_interaction:
    - main operating algo is: activate.py
    - this calls various in directory ROS services.

### 1. Run all sensors:
    e.g., on pi, run camera1Node.py
### 2. Run all services: 
    e.g, in discovery_via_interaction directory, run: python objectDetector.py
### 3. Run the main workflow:
    in discovery_via_interaction directory, run: python activate.py