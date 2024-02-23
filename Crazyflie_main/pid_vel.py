import numpy as np

class PositionPIDController():
    def __init__(self, kp, ki, kd, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = np.clip(output, -self.max_output, self.max_output)
        self.prev_error = error
        return output

class QuadrotorController():
    def __init__(self):
        # Constants for PID controllers
        kp_position = 0.7 #0.5
        ki_position = 0.0 # 0.001 Bajar si hace falta para que no haya tanto error
        kd_position = 0.0 #0.3 No tocar
        max_output = 0.8
        
        # Create PID controllers for each direction
        self.pid_forward = PositionPIDController(kp_position, ki_position, kd_position, max_output)
        self.pid_sideways = PositionPIDController(0, 0, 0, max_output)
        self.pid_yaw = PositionPIDController(1, 0, 0, 1)

    def control_quadrotor(self, current_position, target_position, dt):
        # Calculate errors in each direction
        forward_error = abs(target_position[0] - current_position[0])
        sideways_error = target_position[1] - current_position[1]
        yaw_error = target_position[5] - current_position[5]

        # Use PID controllers to calculate desired velocities
        forward_desired = self.pid_forward.calculate(forward_error, dt)
        sideways_desired = self.pid_sideways.calculate(sideways_error, dt)
        yaw_desired = self.pid_yaw.calculate(yaw_error, dt)        
        
        # Return the calculated desired velocities
        return forward_desired, sideways_desired, yaw_desired
