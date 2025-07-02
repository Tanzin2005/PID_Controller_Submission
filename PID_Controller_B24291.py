class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0
        self.dt = 0.05  # simulation timestep

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        self.integral += error * self.dt
        derivative = (error - self.last_error) / self.dt

        p_term = self.Kp * error
        i_term = self.Ki * self.integral
        d_term = self.Kd * derivative

        output = p_term + i_term + d_term
        self.last_error = error
        self.last_output = output

        return output

    def set_gains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0

    def get_pid_terms(self):
        return {
            "proportional": self.Kp * self.last_error,
            "integral": self.Ki * self.integral,
            "derivative": self.Kd * ((self.last_error - 0) / self.dt),
            "total_output": self.last_output
        }
