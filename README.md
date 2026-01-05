# 🚀 rocket-flight-computer 
## 🤖 What is this?
This is the code for the flight computer that I have made for my course's final project.
It consists of a embedded C software that runs into and ESP32 Microcontroller inside a water bottle rocket.

You see a demonstration video, find more info about this project and how I made it here: !https://www.youtube.com/watch?v=_QPlFLTgm8g
### It's pourpose
To log data in a SD card and transmit sensor readings wirelessly, as well as detecting the rocket's apogee (aka max height of the flight) and deploy a parachute right after it.
### Why
- Transmiting rocket telemtry data is essencial for more complex rocket flights. It's a powerful tool that helps understand the behaviour of the rocket.
- It's pretty cool actually be able to see what's happeing in a so popullar water bottle rocket.
- To demonstrate that it is possible to have this kind of device in any size model rocket.
### replay.py 
This is the custom serial plotter software, that I made to vizualize the telemetry data in real time. Since it was made within 3 days its pretty buggy and has a very poor perfomance due to the MatPlotLib library, but it can do it's job.
To use it, just plug an ESP32 into a USB port on your PC and run the software! It will automatically recognizes it! (must have CP210x drivers installed).
**Features**:
- Dynamically changes the title of some graphs to see the actual max values in real time.
- It has a "replay mode" wich can literally replay the last flight from a `.csv` file.
