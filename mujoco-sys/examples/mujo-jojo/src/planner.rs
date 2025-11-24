use ndarray::{self, Array1};
use ndarray_linalg::Norm;

use crate::pid::PID;

pub struct Planner {
    pub target:    [f64; 3], 
    pub vel_limit: f64,
    pub pidx:      PID,
    pub pidy:      PID,
}

impl Planner {

    pub fn new(target: [f64; 3], vel: f64) -> Self {
        if vel < 0. {
            panic!("planner  velocity must be at least 0");
        }
        let pidx = PID::with_limits(2., 0.15, 1.5, target[0], 0., [Some(-vel),Some(vel)]);
        let pidy = PID::with_limits(2., 0.15, 1.5, target[1], 0., [Some(-vel),Some(vel)]);

        Self { 
            target, 
            vel_limit: vel, 
            pidx,
            pidy,
        }
    }

    pub fn calc(&mut self, loc: &[f64]) -> ndarray::Array1<f64> {
        let mut velocities: Array1<f64> = ndarray::Array1::from_vec(vec![0.,0.,0.]);
        velocities[0] = self.pidx.calc(loc[0], None).unwrap();
        velocities[1] = self.pidy.calc(loc[1], None).unwrap();
        velocities
    }

    pub fn get_alt_setpoint(&self, loc: ndarray::ArrayView1<f64>) -> f64 {
        let dist = self.target[2] - loc[2];
        if dist > 0.5 {
            let timetotarget = dist / self.vel_limit;
            let number_steps = timetotarget / 0.05;
            // compute distance for next update
            let delta_alt = dist / number_steps.round();
            // 4 times for smoothing
            loc[2] + 4. * delta_alt
        } else {
            self.target[2]
        }
    }

    pub fn get_velocities(&self, loc: ndarray::Array1<f64>, target: ndarray::Array1<f64>, flight_speed: Option<f64>) -> ndarray::Array1<f64> {
        let direction = target - loc;
        let dist = direction.norm();
        if dist > 1. {
            let fsp = flight_speed.unwrap_or(0.5);
            fsp  * direction / dist
        } else {
            dist * direction
        }
    }

    pub fn update_target(&mut self, target: [f64;3]) {
        self.target = target;
        self.pidx.setpoint = target[0];
        self.pidy.setpoint = target[1];
    }

}
