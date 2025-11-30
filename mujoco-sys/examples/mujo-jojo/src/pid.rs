use foxglove::Encode;
use serde::Serialize;

#[derive(Debug, Default, Clone, Copy, Serialize, Encode)]
pub struct PidCtx {
    pub proportional: f64,
    pub integral:     f64,
    pub derivative:   f64,
    pub setpoint:     f64,
    pub input:        f64,
    pub output:       f64,
    pub error :       f64,
    pub time:         f64,
}

pub struct PID  {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub setpoint: f64,
    pub sample_time: f64,
    pub outlims: [Option<f64>;2],
    pub auto: bool,
    pub proportional_on_measurement: bool, 
    pub differential_on_measurement: bool,
    pub error_map: fn(f64) -> f64,
    pub time_fn:   Box<dyn Fn() -> f64>,
    pub callback:  Box<dyn Fn(PidCtx)>,

    proportional: f64,
    integral:     f64,
    derivative:   f64,

    last_time:   Option<f64>,
    last_input:  Option<f64>,
    last_output: Option<f64>,
    last_error:  Option<f64>,
}

fn clamp(v: Option<f64>, min: Option<f64>, max: Option<f64>) -> Option<f64> {
    if v.is_none() { return v; }
    let val = v.unwrap();
    match (min, max) {
        (None, None) => v,
        (None, Some(upper)) =>    { if val > upper { Some(upper) } else { v }},
        (Some(lower), None) =>    { if val < lower { Some(lower) } else { v }},
        (Some(lower), Some(upper)) => {  Some(f64::clamp(val, lower, upper)) },
    }
}

impl PID {

    pub fn new(kp: f64, ki: f64, kd: f64, setpoint: f64) -> Self {
        let instant = std::time::Instant::now();
        let mut s = Self {
            kp, ki, kd, setpoint,
            sample_time: 0.01,
            outlims: [None, None],
            auto: true,
            proportional_on_measurement: false, 
            differential_on_measurement: true,
            error_map: |n|{ n },
            time_fn:   Box::new(move || instant.elapsed().as_secs_f64()), 
            callback:  Box::new(|_| {}),

            proportional: 0.,
            integral:     0.,
            derivative:   0.,

            last_time:    None,
            last_input:   None,
            last_output:  None,
            last_error:   None,
        };
        s.reset();
        s.integral = clamp(Some(0.), s.outlims[0], s.outlims[1]).unwrap();
        s
    }

    pub fn set_callback(&mut self, callback: Box<dyn Fn(PidCtx)>) {
        self.callback = callback
    }

    pub fn with_limits(kp: f64, ki: f64, kd: f64, setpoint: f64, sout: f64, limits: [Option<f64>; 2]) -> Self {
        let instant = std::time::Instant::now();
        let mut s = Self {
            kp, ki, kd, setpoint,
            sample_time: 0.01,
            outlims: limits,
            auto: true,
            proportional_on_measurement: false, 
            differential_on_measurement: true,
            error_map: |n|{ n },
            time_fn:   Box::new(move || instant.elapsed().as_secs_f64()), 
            callback:  Box::new(|_| {}),

            proportional: 0.,
            integral:     clamp(Some(sout), limits[0], limits[1]).unwrap(),
            derivative:   0.,

            last_time:    None,
            last_input:   None,
            last_output:  None,
            last_error:   None,
        };
        s.reset();
        s.integral = clamp(Some(sout), s.outlims[0], s.outlims[1]).unwrap();
        s
    }

    pub fn limits(&mut self, min: f64, max: f64)  {
        if max < min {
            panic!("lower limit cannot exceed upper");
        }
        self.outlims     = [Some(min), Some(max)];
        self.integral    = clamp(Some(self.integral), Some(min), Some(max)).unwrap();
        self.last_output = clamp(self.last_output, Some(min), Some(max));
    }

    /** 
    The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
    for visualizing what the controller is doing or when tuning hard-to-tune systems.
    */
    pub fn components(&self) -> (f64, f64,f64) {
        (self.proportional, self.integral, self.derivative)
    }

    /** 
    Enable or disable the PID controller, optionally setting the last output value.
    This is useful if some system has been manually controlled and if the PID should take over.
    In that case, disable the PID by setting auto mode to False and later when the PID should
    be turned back on, pass the last output variable (the control variable) and it will be set
    as the starting I-term when the PID is set to auto mode.

    :param enabled: Whether auto mode should be enabled, True or False
    :param last_out: The last output, or the control variable, that the PID should start
        from when going from manual mode to auto mode. Has no effect if the PID is already in
        auto mode.
    */
    pub fn toggleauto(&mut self, enabled: bool, last_out: Option<f64>) {
        if enabled && !self.auto {
            self.reset();
            self.integral = last_out.unwrap_or(0.);
            self.integral = clamp(Some(self.integral), self.outlims[0], self.outlims[1]).unwrap();
        }
        self.auto = enabled
    }

    /**
    *Reset the PID controller internals.
    This sets each term to 0 as well as clearing the integral, the last output and the last
    input (derivative calculation).
    * */
    pub fn reset(&mut self)  { 
        self.proportional = 0.;
        self.integral     = 0.;
        self.derivative   = 0.;

        self.integral = clamp(Some(self.integral), self.outlims[0], self.outlims[1]).unwrap();

        self.last_time   = Some((self.time_fn)());
        self.last_output = None;
        self.last_input  = None;
        self.last_error  = None
    }

    pub fn calc(&mut self, input: f64, delta: Option<f64>) -> Option<f64> {
        if !self.auto {
            return self.last_output
        }

        let now = (self.time_fn)();
        let dt = delta.unwrap_or({
            match self.last_time {
                Some(last) => {
                    now - last
                }
                None => {
                    1e-16
                }
            }
        });

        if dt < self.sample_time && self.last_output.is_some() {
            // Only update every sample_time seconds
            (self.callback)(
                PidCtx { 
                    proportional: self.proportional, 
                    integral:     self.integral, 
                    derivative:   self.derivative, 
                    setpoint:     self.setpoint, 
                    input, 
                    output:       self.last_output.unwrap_or(-1.),  // negative numbers to show unset values
                    error:        self.last_error.unwrap_or(-1.),
                    time:         self.last_time.unwrap_or(-1.)
                }
            );
            return self.last_output
        }

        // Compute error terms
        let mut error = self.setpoint - input;
        let d_input   = input - (self.last_input.unwrap_or(input));
        let d_error   = error - (self.last_error.unwrap_or(error));

        // Check if must map the error
        error = (self.error_map)(error);

        // Compute the proportional term
        if !self.proportional_on_measurement {
            // Regular proportional-on-error, simply set the proportional term
            self.proportional =  (self.kp * error);
        } else {
            // Add the proportional error on measurement to error_sum
            self.proportional -= (self.kp * d_input);
        }

        // Compute integral and derivative terms
        self.integral += (self.ki * error * dt);
        self.integral = clamp(Some(self.integral), self.outlims[0], self.outlims[1]).unwrap();

        if self.differential_on_measurement {
            self.derivative = -self.kd * d_input / dt;
        } else {
            self.derivative =  self.kd * d_error / dt;
        }

        // Compute final output
        let mut output = self.proportional + self.integral + self.derivative;
        output = clamp(Some(output), self.outlims[0], self.outlims[1]).unwrap();

        // Keep track of state
        self.last_output = Some(output);
        self.last_input  = Some(input);
        self.last_error  = Some(error);
        self.last_time   = Some((self.time_fn)());

        (self.callback)(
            PidCtx { 
                proportional: self.proportional, 
                integral:     self.integral, 
                derivative:   self.derivative, 
                setpoint:     self.setpoint, 
                input, 
                output:       self.last_output.unwrap_or(-1.),  // negative numbers to show unset values
                error:        self.last_error.unwrap_or(-1.),
                time:         self.last_time.unwrap_or(-1.)
            }
        );
        Some(output)
    }

}
