
class PID:
    def Init(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.p_error = 0
        self.i_error = 0
        self.d_error = 0

        return

    def UpdateError(self, cte):
        self.i_error += cte
        self.d_error = cte - self.p_error
        self.p_error = cte

        return

    def TotalError(self):
        error = self.Kp * self.p_error +\
                self.Kd * self.d_error +\
                self.Ki * self.i_error

        return error