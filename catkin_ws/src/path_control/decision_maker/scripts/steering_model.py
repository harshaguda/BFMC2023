import math

MAX_ANGLE = 24.0

class AckermannModel():
    def __init__(self, steering_offset=0):
        self.wheel_track  = 16.2
        self.wheel_base   = 26.0
        self.pivot_dist   = 12.0
        self.steering_offset = steering_offset
        
    def ackerman_angle(self):
        self.ackerman_angle_in_rads = math.atan(self.pivot_dist/2/wheel_base)
        self.ackerman_angle = math.degrees(self.ackerman_angle_in_rads)
        print("The ackerman angle is {}".format(self.ackerman_angle))
        
    def calculate_steering_command(self, curvature):
        delta_in  = math.atan(self.wheel_base/(curvature - 0.5*self.wheel_track)
        delta_out = math.atan(self.wheel_base/(curvature + 0.5*self.wheel_track)
        
        steering_angle = 0.5*(delta_in + delta_out)
        steering_angle = math.degrees(steering_angle)
        
        ## Adjusting the offset
        steering_angle += self.steering_offset
        
        if abs(steering_angle) > MAX_ANGLE:
            steering_angle = MAX_ANGLE if steering_angle > 0 else -1*MAX_ANGLE
        return steering_angle
