# !/home/mahdi255/.pyenv/shims/python

class Pid:

    def __init__(self, name, k_p, k_i, k_d, dt) -> None:
        self.name = name

        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.dt = dt
        self.prev_error = 0

        self.P = 0
        self.I = 0
        self.D = 0

        self.sum_i_theta = 0

        self.errs = []

    def calculate(self, err):
        self.errs.append(err)
        self.sum_i_theta += err * self.dt

        self.P = self.k_p * err
        self.I = self.k_i * self.sum_i_theta
        self.D = self.k_d * (err - self.prev_error)

        self.prev_error = err

        return self.P + self.I + self.D
