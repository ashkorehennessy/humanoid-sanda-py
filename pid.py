class pid:
    def __init__(self):
        self.Kp = 0.3
        self.Kd = 0
        self.last_error = 0
        self.output_max = 100
        self.output_min = -100

    def update(self, input_value, setpoint):
        error = setpoint - input_value
        derivative = error - self.last_error
        self.last_error = error
        output = max(min(self.Kp * error + self.Kd * derivative, self.output_max), self.output_min)
        return int(output)