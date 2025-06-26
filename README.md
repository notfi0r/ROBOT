Use a mobile hotspot to make sure no network firewalls interfere with the ESP to webots connection
connect both the device with webots and the esp32 (in the code) to the same network
Webots installed (tested with version R2023b or later)
Python API for Webots (included with Webots installation)
MicroPython installed on the ESP32
Thonny IDE or any MicroPython-compatible IDE

Required MicroPython libraries (all come preinstalled with MicroPython):
network
socket
machine
ujson
time

How to run:
Connect both the ESP32 and the computer (running Webots) to the same Wi-Fi network.
Upload and run the ESP32 script using Thonny or any MicroPython IDE. Wait for the ESP32 to display "Waiting for Webots connection...".
Update the ESP32 IP address in the Webots Python controller script:
ESP32_IP = 'your_esp32_ip'
Open the Webots project and run the simulation. The Webots robot controller will connect to the ESP32 over Wi-Fi.
The ESP32 will handle path planning and send movement commands to Webots. Webots executes the movements and sends back sensor data and status updates.

Notes:
Ensure port 8888 is open and not blocked by firewalls.
The Webots robot controller requires the ESP32 to be running before it starts.
Update the start and goal nodes in the Webots script as needed.
The graph map and node connections are defined in the ESP32 script.
