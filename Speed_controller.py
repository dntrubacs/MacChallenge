"""
 Created by CHihiro Hirai for the MacChellenge team. The purpose of this modules to set up a simple PID controller for the boat.
"""


#Speed controller
def clamp(value, limits):
    """Clamps an input value using a tuple of limits

    Parameters
    ----------
    value : float
        The value to clamp.
    limits : tuple(float, float)
        The lower and upper limits.

    Returns
    -------
    float
        Returns the same value if between limits, returns the limit if out of boundaries.
    """
    lower, upper = limits
    if value is None:
        return None
    elif upper is not None and value > upper:
        return upper
    elif lower is not None and value < lower:
        return lower
    return value

class PID:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, setpoint=0.0, limits=None):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.previous_input = 0.0
        self.proportional = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.limits = limits
        
    def call(self, measured_value, dt):
        
        error = self.setpoint - measured_value
        prev_error = self.setpoint - self.previous_input
        
        self.proportional = self.Kp * error
        area = (prev_error + error)/2*dt
        self.integral = self.Ki * area * dt
        self.derivative = self.Kd* (prev_error-error) /dt
        
        #self.integral = clamp(self.integral, self.limits)
        output = clamp(self.proportional + self.integral +  self.derivative, self.limits)
        self.previous_input = measured_value
        return output
    
    
    
    
    
    
if __name__ == '__main__':
    controler = PID(Kp=1.0,Ki=1.0,Kd=0.0,setpoint=5.0,limits=(-100,100))
    print(controler.call(3))



