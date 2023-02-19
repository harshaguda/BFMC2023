def generate_command(action, value):
    command = {"action": '0'}
    command["action"] = str(action)
    
    if (command["action"] == "1"): # Vehicle Speed
        command["speed"] = float(value)
    elif (command["action"] == "2"): # Vehicle Steering angle
        command["steerAngle"] = float(value)
    elif (command["action"] == "3"): # Vehicle Brake command
        command["steerAngle"] = float(value)
    elif (command["action"] == "4"): # Activate PID control
        command["activate"] = bool(value)
    elif (command["action"] == "5"): # Activate Encoder publisher
        command["activate"] = bool(value)
    elif (command["action"] == "6"): # Pass PID Values
        command["kp"] = float(value[0])
        command["ki"] = float(value[1])
        command["kd"] = float(value[2])
        command["tf"] = float(value[3])
    else:
        print("Invalid command")
        command = {"action": '0'}
    return command
            
