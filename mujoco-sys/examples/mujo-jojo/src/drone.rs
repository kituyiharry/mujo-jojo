use std::{cell::{RefCell}, slice};

use crate::{pid, planner};

pub struct Drone {
    pub model  : *mut mujoco_sys::mjModel,
    pub data   : *mut mujoco_sys::mjData,
    pub planner: RefCell<planner::Planner>,

    // inner control
    pid_alt:   RefCell<pid::PID>,
    pid_roll:  RefCell<pid::PID>,
    pid_pitch: RefCell<pid::PID>,
    pid_yaw:   RefCell<pid::PID>,

    // outer control
    pid_v_x:   RefCell<pid::PID>,
    pid_v_y:   RefCell<pid::PID>,
}

impl Drone {
    pub fn new(model: *mut mujoco_sys::mjModel, data: *mut mujoco_sys::mjData, target: [f64;3]) -> Self {

        // magically precalculated tolerances
        
        let rollpid   = pid::PID::with_limits(2.6785,0.56871, 1.2508, 0., 0., [Some(-1.), Some(1.)]);
        let pitchpid  = pid::PID::with_limits(2.6785,0.56871, 1.2508, 0., 0., [Some(-1.), Some(1.)]);
        let yawpid    = pid::PID::with_limits(0.54, 0., 5.358333, 1., 0.,     [Some(-3.), Some(3.)]);

        let pidvx   = pid::PID::with_limits(0.1, 0.003, 0.02, 0., 0., [Some(-0.1), Some(0.1)]);
        let pidvy   = pid::PID::with_limits(0.1, 0.003, 0.02, 0., 0., [Some(-0.1), Some(0.1)]);

        Self { 
            model, 
            data, 

            planner:   RefCell::new(planner::Planner::new(target, 2.)), 
            pid_alt:   RefCell::new(pid::PID::new(5.50844,0.57871, 1.2, 0.)),

            pid_roll:  RefCell::new(rollpid), 
            pid_pitch: RefCell::new(pitchpid),
            pid_yaw:   RefCell::new(yawpid), 

            pid_v_x:   RefCell::new(pidvx), 
            pid_v_y:   RefCell::new(pidvy), 
        }
    } 

    #[inline]
    fn velocity(&self) -> &[f64] {
        unsafe {
            slice::from_raw_parts_mut((*self.data).qvel, 6)
        }
    }

    #[inline]
    fn position(&self) -> &[f64] {
        unsafe {
            slice::from_raw_parts_mut((*self.data).qpos, 7)
        }
    }

    #[inline]
    fn acceleration(&self) -> &[f64] {
        unsafe {
            slice::from_raw_parts_mut((*self.data).qacc, 6)
        }
    }

    pub fn outer(&mut self) {
        let vel  = self.velocity();
        let loc  = self.position();
        //let acc  = self.acceleration();
        //println!("===========================");
        //println!("{vel:?}");
        //println!("{loc:?}");
        //println!("{acc:?}");
        //println!("===========================");
        let velocities = self.planner.borrow_mut().calc(&loc[0..3]);
        self.pid_alt.borrow_mut().setpoint = self.planner.borrow_mut().get_alt_setpoint(
            ndarray::ArrayView1::from_shape(3, &loc[0..3]).unwrap()
        );
        self.pid_v_x.borrow_mut().setpoint = velocities[0];
        self.pid_v_y.borrow_mut().setpoint = velocities[1];

        let angle_pitch = self.pid_v_x.borrow_mut().calc(vel[0], None).unwrap();
        let angle_roll  = self.pid_v_y.borrow_mut().calc(vel[1], None).unwrap();

        self.pid_pitch.borrow_mut().setpoint = angle_pitch;
        self.pid_roll.borrow_mut().setpoint  = angle_roll;
    }

    fn thrusts(&self, thrust: f64, roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
        [
            thrust + roll + pitch  - yaw,
            thrust - roll + pitch  + yaw,
            thrust - roll -  pitch - yaw,
            thrust + roll - pitch  + yaw
        ]
    }

    pub fn inner(&mut self) {
        let pos =    self.position();
        let alt =    &pos[2];
        let angles = &pos[3..6]; // roll, yaw, pitch
        
        // apply PID
        let cmd_thrust =   self.pid_alt.borrow_mut().calc(*alt, None).unwrap() + 3.2495;
        let cmd_roll   = - self.pid_roll.borrow_mut().calc(angles[1], None).unwrap();
        let cmd_pitch  =   self.pid_pitch.borrow_mut().calc(angles[2], None).unwrap();
        let cmd_yaw    = - self.pid_yaw.borrow_mut().calc(angles[0], None).unwrap();

        // transfer to motor control
        let out = self.thrusts(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw);

        unsafe  {
            (*(*self.data).ctrl.offset(0)) = out[0];
            (*(*self.data).ctrl.offset(1)) = out[1];
            (*(*self.data).ctrl.offset(2)) = out[2];
            (*(*self.data).ctrl.offset(3)) = out[3];
        }
    }
}
