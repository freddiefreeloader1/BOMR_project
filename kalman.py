class Kalman:
    p = 0
    i = 0
    d = 0
    last_current = 0
    i_sum = 0
    def __init__(self, p, i , d):
        self.p = p
        self.i = i
        self.d = d
    def get(self, current,setpoint):
        feedback = (setpoint - current) * self.pg - (current - self.last_current)*self.d + (self.i_sum * self.i)

        i_sum += (current-self.last_current)
        self.last_current = current

        return feedback