from controller import Robot
import json
import socket
import time

# --- Socket TCP client to ESP32 ---
ESP32_IP = ''  # your ip
ESP32_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(5)
try:
    sock.connect((ESP32_IP, ESP32_PORT))
    print("Connected to ESP32")
except Exception as e:
    print("Connection failed:", e)
    exit(1)
#sock.settimeout(0.05)
sock.settimeout(0.3)

# --- Webots Initialization ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED
counter = 0
COUNTER_MAX = 5

#Pass these for Esp32 to build path 

start_node = 'B4'
goal_node = 'A1'

# states
states = ['forward', 'forward_bit', 'swing_right', 'swing_left', 'turn_right', 'turn_left', 'turn_back', 'stop','nothing']
current_state = 'forward'
message =''

gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)
    
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# --- Wait for ESP32 ready ---
esp32_ready = False
while robot.step(timestep) != -1 and not esp32_ready:
    try:
        msg = sock.recv(1024).decode().strip()
        if msg == 'ready':
            sock.send((json.dumps({'start': start_node, 'goal': goal_node}) + '\n').encode())
            esp32_ready = True
    except:
        pass

while robot.step(timestep) != -1:
    # 1. Always read sensors and send message to ESP32
    gsValues = [gs[i].getValue() for i in range(3)]
    psValues = [ps[i].getValue() for i in range(8)]
     
    message = ''.join(['0' if v > 600 else '1' for v in gsValues]) + '\n'
    obstacle_detected = psValues[0] > 300 or psValues[7] > 300
    
    if obstacle_detected:
        sock.send(b'obstacle\n')
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(-0.5 * speed)
            rightMotor.setVelocity(0.5 * speed)
            if robot.getTime() - turn_start >= 3.52:
                break
        try:
            while True:
                response = sock.recv(1024).decode().strip()
                if response == 'replanned':
                    print("[INFO] ESP32 finished re-planning")
                    break
        except socket.timeout:
            print("Timeout waiting for re-planning confirmation")
            
        continue
    try:
        sock.send(message.encode())
    except:
        print("Send failed, ESP32 disconnected?")
        break
    try:
        msg = sock.recv(1024).decode().strip()
        if msg in states:
            current_state = msg
        elif msg:
            print("ESP32 says:", msg)
    except Exception as e:
        pass
        
    line_right = gsValues[0] > 600
    line_center = gsValues[1] > 600
    line_left = gsValues[2] > 600
  
    # 3. React based on current state
    if line_right and not line_left:
        current_state = 'swing_right'
       
    elif line_left and not line_right:
        current_state = 'swing_left' 
        
    elif line_left and line_right and line_center: # lost the line
        current_state = 'swing_left'       
            
    if current_state == 'forward':
        counter = 0
        leftSpeed = speed
        rightSpeed = speed
            
    if current_state == 'swing_right':
        leftSpeed = 0.5 * speed
        rightSpeed = 0 * speed
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            
   
    if current_state == 'swing_left':
        leftSpeed = 0 * speed
        rightSpeed = 0.5 * speed
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            
    if current_state == 'stop':
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
    counter += 1
      
    if current_state == 'forward_bit':
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            if robot.getTime() - turn_start >= 0.8:
                break
        current_state = 'forward'
        sock.send(b'done\n')

    elif current_state == 'turn_right':
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            if robot.getTime() - turn_start >= 0.6:
                break
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(0.5 * speed)
            rightMotor.setVelocity(-0.5 * speed)
            if robot.getTime() - turn_start >= 1.76:
                break
        current_state = 'forward'
        sock.send(b'done\n')
        
    elif current_state == 'turn_left':
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            if robot.getTime() - turn_start >= 0.6:
                break
        turn_start = robot.getTime()
        while robot.step(timestep) != -1:
            leftMotor.setVelocity(-0.5 * speed)
            rightMotor.setVelocity(0.5 * speed)
            if robot.getTime() - turn_start >= 1.76:
                break
        current_state = 'forward'
        sock.send(b'done\n')
   
    print(f"Sensor: {message.strip()} - State: {current_state}")