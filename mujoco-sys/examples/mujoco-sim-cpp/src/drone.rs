use std::{cell::{Cell, RefCell}, rc::Rc, slice};

use serde::Serialize;

use crate::{pid::{self, PidCtx}, planner::{self, PlannerCtx}};

pub struct Drone {
    pub model  : *mut mujoco_sys::mjModel,
    pub data   : *mut mujoco_sys::mjData,
    pub planner: RefCell<planner::Planner>,

    // inner control
    pid_alt:   pid::PID,
    pid_roll:  pid::PID,
    pid_pitch: pid::PID,
    pid_yaw:   pid::PID,

    // outer control
    pid_v_x:   pid::PID,
    pid_v_y:   pid::PID,

    // save modifications here
    pub pidactx:  Rc<Cell<PidCtx>>,
    pub pidrctx:  Rc<Cell<PidCtx>>,
    pub pidpctx:  Rc<Cell<PidCtx>>,
    pub pidyctx:  Rc<Cell<PidCtx>>,

    pub pidvxctx: Rc<Cell<PidCtx>>,
    pub pidvyctx: Rc<Cell<PidCtx>>,
    pub plannerc: Rc<Cell<PlannerCtx>>,

    callback:  Box<dyn Fn(DroneCtx)>
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct DroneCtx {

    pub planner: PlannerCtx,

    pub pidactx:  PidCtx,
    pub pidroll:  PidCtx,
    pub pidptch:  PidCtx,
    pub pidyaw:   PidCtx,

    pub pidvx:    PidCtx,
    pub pidvy:    PidCtx,

    pub velocity: [f64;6],
    //??? Not encodable by foxglove::Encode ????!!!!!!
    //pub linearv:   Vector3, 
    //pub anglvel:   Vector3, 

    pub position: [f64;7],
    //pub frame:    Point3,
    //pub quat:     Quaternion,

    pub accelrtn: [f64;6],
    //pub linracc: Vector3,
    //pub anglacc: Vector3,

    pub thrust:   f64,
    pub pitch:    f64,
    pub roll:     f64,
    pub yaw:      f64,

}

impl Drone {
    pub fn new(model: *mut mujoco_sys::mjModel, data: *mut mujoco_sys::mjData, target: [f64;3]) -> Self {

        // magically precalculated tolerances
        
        let mut rollpid   = pid::PID::with_limits(2.6785,0.56871, 1.2508,  0., 0., [Some(-1.), Some(1.)]);
        let mut pitchpid  = pid::PID::with_limits(2.6785,0.56871, 1.2508,  0., 0., [Some(-1.), Some(1.)]);
        let mut yawpid    = pid::PID::with_limits(0.54,  0.,      5.358333,1., 0., [Some(-3.), Some(3.)]);

        let mut pidvx     = pid::PID::with_limits(0.1, 0.003, 0.02, 0., 0., [Some(-0.01), Some(0.01)]);
        let mut pidvy     = pid::PID::with_limits(0.1, 0.003, 0.02, 0., 0., [Some(-0.01), Some(0.01)]);

        pidvx.sample_time = 0.01;
        pidvy.sample_time = 0.01;

        let mut pidalt    = pid::PID::new(5.50844,0.57871, 1.2, 0.);

        //let mut pidalt    = pid::PID::with_limits(1.0, 0.5, 1.2, 0., 0., [Some(0.), None]);
        //let mut rollpid   = pid::PID::with_limits(1.,  0.1,  1.,  0., 0., [Some(-1.), Some(1.)]);
        //let mut pitchpid  = pid::PID::with_limits(1.,  0.,   0.,  0., 0., [Some(-1.), Some(1.)]);
        //let mut yawpid    = pid::PID::with_limits(1.,  0.1,  0.1,  1.0, 0., [Some(-3.), Some(3.)]);

        //let mut pidvx     = pid::PID::with_limits(1., 0.1, 0.1, 0., 0., [Some(-0.1), Some(0.1)]);
        //let mut pidvy     = pid::PID::with_limits(1., 0.1, 0.1, 0., 0., [Some(-0.1), Some(0.1)]);

        let pidactx:   Rc<Cell<PidCtx>> = Rc::default();
        let pidacln  = Rc::clone(&pidactx);
        let pidrctx:   Rc<Cell<PidCtx>> = Rc::default();
        let pidrcln  = Rc::clone(&pidrctx);
        let pidpctx:   Rc<Cell<PidCtx>> = Rc::default();
        let pidpcln  = Rc::clone(&pidpctx);
        let pidyctx:   Rc<Cell<PidCtx>> = Rc::default();
        let pidycln  = Rc::clone(&pidyctx);
        let pidvxctx:  Rc<Cell<PidCtx>> = Rc::default();
        let pidvxcln = Rc::clone(&pidvxctx);
        let pidvyctx:  Rc<Cell<PidCtx>> = Rc::default();
        let pidvycln = Rc::clone(&pidvyctx);
        let plannerc: Rc<Cell<PlannerCtx>> = Rc::default();
        let plnrclne = Rc::clone(&plannerc);

        pidalt.set_callback(Box::new(move |lastctx| {
            pidacln.set(lastctx);
        }));
        rollpid.set_callback(Box::new(move |lastctx| {
            pidrcln.set(lastctx);
        }));
        pitchpid.set_callback(Box::new(move |lastctx| {
            pidpcln.set(lastctx);
        }));
        yawpid.set_callback(Box::new(move |lastctx| {
            pidycln.set(lastctx);
        }));
        pidvx.set_callback(Box::new(move |lastctx| {
            pidvxcln.set(lastctx);
        }));
        pidvy.set_callback(Box::new(move |lastctx| {
            pidvycln.set(lastctx);
        }));

        let mut planner =  planner::Planner::new(target, 1.);
        planner.set_callback(Box::new(move |lastctx|{
            plnrclne.set(lastctx);
        }));

        Self { 
            model, 
            data, 

            planner:   RefCell::new(planner), 
            pid_alt:   pidalt,

            pid_roll:  rollpid, 
            pid_pitch: pitchpid,
            pid_yaw:   yawpid, 

            pid_v_x:   pidvx, 
            pid_v_y:   pidvy, 

            pidactx,
            pidrctx,
            pidpctx,
            pidyctx,
            pidvxctx,
            pidvyctx,
            plannerc,

            callback:  Box::new(|_|{})
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

    pub fn set_callback(&mut self, callback: Box<dyn Fn(DroneCtx)>) {
        self.callback = callback
    }

    pub fn outer(&mut self) {
        unsafe {
            let vel  = slice::from_raw_parts_mut((*self.data).qvel, 6);
            let loc  = slice::from_raw_parts_mut((*self.data).qpos, 7);
            let velocities = self.planner.borrow_mut().calc(&loc[0..3]);
            self.pid_alt.setpoint = self.planner.borrow_mut().get_alt_setpoint(
                ndarray::ArrayView1::from_shape(3, &loc[0..3]).unwrap()
            );

            self.pid_v_x.setpoint = velocities[0];
            self.pid_v_y.setpoint = velocities[1];

            let angle_pitch = self.pid_v_x.calc(vel[0], None).unwrap();
            let angle_roll  = self.pid_v_y.calc(vel[1], None).unwrap();
            self.pid_pitch.setpoint = angle_pitch;
            self.pid_roll.setpoint  = angle_roll;

            let mut v = [0.;6];
            let mut p = [0.;7];
            let mut a = [0.;6];

            let acc = self.acceleration();

            v.copy_from_slice(vel);
            p.copy_from_slice(loc);
            a.copy_from_slice(acc);

            (self.callback)(DroneCtx { 
                planner:   self.plannerc.get(), 
                pidactx:   self.pidactx.get(),
                pidroll:   self.pidrctx.get(), 
                pidptch:   self.pidpctx.get(), 
                pidyaw:    self.pidyctx.get(), 
                pidvx:     self.pidvxctx.get(), 
                pidvy:     self.pidvyctx.get(), 
                velocity: v,
                position: p,
                accelrtn: a,
                //linearv:   Vector3 { x: vel[0], y: vel[1], z: vel[2] },
                //anglvel:   Vector3 { x: vel[3], y: vel[4], z: vel[5] },
                //frame  :   Point3  { x: loc[0],  y: loc[1], z: loc[2] },
                //quat:      Quaternion { x: loc[3], y: loc[4], z: loc[5], w: loc[6] },
                //linracc:   Vector3 { x: acc[0], y: acc[1], z: acc[2] }, 
                //anglacc:   Vector3 { x: acc[3], y: acc[4], z: acc[5] }, 
                thrust:     0.,
                pitch:      0., 
                roll:       0.,
                yaw:        0.
            });

        }
    }

    fn thrusts(&self, thrust: f64, roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
        [
            thrust + roll + pitch  - yaw,
            thrust - roll + pitch  + yaw,
            thrust - roll - pitch  - yaw,
            thrust + roll - pitch  + yaw
        ]
    }

    pub fn inner(&mut self) {
        unsafe {
            let pos = slice::from_raw_parts_mut((*self.data).qpos, 7);
            //self.position();
            let alt =    &pos[2];
            let angles = &pos[3..6];

            // apply PID
            let cmd_thrust =   self.pid_alt.calc(       *alt, None).unwrap() + 3.2495;
            let cmd_roll   = - self.pid_roll.calc( angles[1], None).unwrap();
            let cmd_pitch  =   self.pid_pitch.calc(angles[2], None).unwrap();
            let cmd_yaw    = - self.pid_yaw.calc(  angles[0], None).unwrap();

            // transfer to motor control
            let out = self.thrusts(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw);

            (*(*self.data).ctrl.offset(0)) = out[0];
            (*(*self.data).ctrl.offset(1)) = out[1];
            (*(*self.data).ctrl.offset(2)) = out[2];
            (*(*self.data).ctrl.offset(3)) = out[3];

            let mut v = [0.;6];
            let mut p = [0.;7];
            let mut a = [0.;6];

            let acc = self.acceleration();
            let vel = self.velocity();

            v.copy_from_slice(vel);
            p.copy_from_slice(pos);
            a.copy_from_slice(acc);

            (self.callback)(DroneCtx { 
                planner:  self.plannerc.get(), 
                pidactx:  self.pidactx.get(),
                pidroll:  self.pidrctx.get(), 
                pidptch:  self.pidpctx.get(), 
                pidyaw:   self.pidyctx.get(), 
                pidvx:    self.pidvxctx.get(), 
                pidvy:    self.pidvyctx.get(), 
                velocity: v,
                position: p,
                accelrtn: a,
                //linearv:  Vector3 { x: vel[0], y: vel[1], z: vel[2] },
                //anglvel:  Vector3 { x: vel[3], y: vel[4], z: vel[5] },
                //frame  :  Point3  { x: pos[0],  y: pos[1], z: pos[2] },
                //quat:     Quaternion { x: pos[3], y: pos[4], z: pos[5], w: pos[6] },
                //linracc:  Vector3 { x: acc[0], y: acc[1], z: acc[2] }, 
                //anglacc:  Vector3 { x: acc[3], y: acc[4], z: acc[5] }, 
                thrust:     cmd_thrust,
                pitch:      cmd_pitch, 
                roll:       cmd_roll,
                yaw:        cmd_yaw,
            });
        }
    }
}
