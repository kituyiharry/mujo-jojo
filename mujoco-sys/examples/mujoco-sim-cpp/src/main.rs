use std::{ptr, thread::{self, sleep, sleep_ms}, time::Duration};
use mujoco_sys::*;

mod drone;
mod pid;
mod planner;

static mut SIM  : *mut mujoco_Simulate = ptr::null_mut();
static mut PERT : *mut mujoco_sys::mjvPerturb = &mut mjvPerturb {
    select: 0,
    flexselect: 0,
    skinselect: 0,
    active: 0,
    active2: 0,
    refpos: [0.;3],
    refquat: [0.;4],
    refselpos: [0.;3],
    localpos: [0.;3],
    localmass: 0.,
    scale: 0. 
};
static mut CAM  : *mut mujoco_sys::mjvCamera = &mut mjvCamera{ 
    type_:        0, 
    fixedcamid:   0, 
    trackbodyid:  0, 
    lookat:       [0.; 3], 
    distance:     0., 
    azimuth:      0., 
    elevation:    0., 
    orthographic: 0 
};                      // abstract camera
static mut OPT  : *mut mujoco_sys::mjvOption = &mut mjvOption{
    label:          0,
    frame:          0,
    geomgroup:      [0u8; 6],
    sitegroup:      [0u8; 6],
    jointgroup:     [0u8; 6],
    tendongroup:    [0u8; 6],
    actuatorgroup:  [0u8; 6],
    flexgroup:      [0u8; 6],
    skingroup:      [0u8; 6],
    flags:          [0u8; 31],
    bvh_depth:      0,
    flex_layer:     0,
};                      // visualization options
static mut SCN  : *mut mujoco_sys::mjvScene  = &mut mjvScene {
    maxgeom:      0,
    ngeom:        0,
    geoms       : &mut mjvGeom_{
        type_       : 0,
        dataid      : 0,
        objtype     : 0,
        objid       : 0,
        category    : 0,
        matid       : 0,
        texcoord    : 0,
        segid       : 0,
        size        : [0.; 3],
        pos         : [0.; 3],
        mat         : [0.; 9],
        rgba        : [0.; 4],
        emission    : 0.,
        specular    : 0.,
        shininess   : 0.,
        reflectance : 0.,
        label       : [0i8; 100],
        camdist     : 0.,
        modelrbound : 0.,
        transparent : 0
    },
    geomorder:    ptr::null_mut(),
    nflex:        0,
    flexedgeadr:  ptr::null_mut(),
    flexedgenum:  ptr::null_mut(),
    flexvertadr:  ptr::null_mut(),
    flexvertnum:  ptr::null_mut(),
    flexfaceadr:  ptr::null_mut(),
    flexfacenum:  ptr::null_mut(),
    flexfaceused: ptr::null_mut(),
    flexedge:     ptr::null_mut(),
    flexvert:     ptr::null_mut(),
    flexface:     ptr::null_mut(),
    flexnormal:   ptr::null_mut(),
    flextexcoord: ptr::null_mut(),
    flexvertopt:  0u8,
    flexedgeopt:  0u8,
    flexfaceopt:  0u8,
    flexskinopt:  0u8,
    nskin:        0,
    skinfacenum:  ptr::null_mut(),
    skinvertadr:  ptr::null_mut(),
    skinvertnum:  ptr::null_mut(),
    skinvert:     ptr::null_mut(),
    skinnormal:   ptr::null_mut(),
    nlight:       0,
    lights      : [mjvLight_{
        id          : 0,
        pos         : [0.;3],
        dir         : [0.;3],
        type_       : 0,
        texid       : 0,
        attenuation : [0.;3],
        cutoff      : 0.,
        exponent    : 0.,
        ambient     : [0.;3],
        diffuse     : [0.;3],
        specular    : [0.;3],
        headlight   : 0,
        castshadow  : 0,
        bulbradius  : 0.,
        intensity   : 0.,
        range       : 0.
    };100],
    camera         : [mjvGLCamera_{
        pos            : [0.;3],
        forward        : [0.;3],
        up             : [0.;3],
        frustum_center : 0.,
        frustum_width  : 0.,
        frustum_bottom : 0.,
        frustum_top    : 0.,
        frustum_near   : 0.,
        frustum_far    : 0.,
        orthographic   : 0,
    }; 2],
    enabletransform: 0,
    translate:  [0.;3],
    rotate:     [0.;4],
    scale:      0.,
    stereo:     0,
    flags:      [0u8;10],
    framewidth: 0,
    framergb:   [0.;3],
    status:     0,
};                      // abstract scene
static mut CON  : *mut mujoco_sys::mjrContext = &mut mjrContext{
    lineWidth          : 0.,
    shadowClip         : 0.,
    shadowScale        : 0.,
    fogStart           : 0.,
    fogEnd             : 0.,
    fogRGBA            : [0.;4],
    shadowSize         : 0,
    offWidth           : 0,
    offHeight          : 0,
    offSamples         : 0,
    fontScale          : 0,
    auxWidth           : [0;10] ,
    auxHeight          : [0;10] ,
    auxSamples         : [0;10] ,
    offFBO             : 0,
    offFBO_r           : 0,
    offColor           : 0,
    offColor_r         : 0,
    offDepthStencil    : 0,
    offDepthStencil_r  : 0,
    shadowFBO          : 0,
    shadowTex          : 0,
    auxFBO             : [0;10],
    auxFBO_r           : [0;10],
    auxColor           : [0;10],
    auxColor_r         : [0;10],
    mat_texid          : [0;10000],
    mat_texuniform     : [0;1000],
    mat_texrepeat      : [0.;2000],
    ntexture           : 0,
    textureType        : [0;1000],
    texture            : [0;1000],
    basePlane          : 0,
    baseMesh           : 0,
    baseHField         : 0,
    baseBuiltin        : 0,
    baseFontNormal     : 0,
    baseFontShadow     : 0,
    baseFontBig        : 0,
    rangePlane         : 0,
    rangeMesh          : 0,
    rangeHField        : 0,
    rangeBuiltin       : 0,
    rangeFont          : 0,
    nskin              : 0,
    skinvertVBO        : ptr::null_mut(),
    skinnormalVBO      : ptr::null_mut(),
    skintexcoordVBO    : ptr::null_mut(),
    skinfaceVBO        : ptr::null_mut(),
    charWidth          : [0;127],
    charWidthBig       : [0;127],
    charHeight         : 0,
    charHeightBig      : 0,
    glInitialized      : 0,
    windowAvailable    : 0,
    windowSamples      : 0,
    windowStereo       : 0,
    windowDoublebuffer : 0,
    currentBuffer      : 0,
    readPixelFormat    : 0,
    readDepthMap       : 0
};                     // custom GPU context

#[cxx::bridge]
mod ffi {
    unsafe extern "C++" {
        include!("bridge.h");
        include!("mujoco/mujoco.h");
        type Simulate;
        type mjvCamera;
        type mjvOption;
        type mjvPerturb;
        type mjData;
        type mjModel;
        unsafe fn createSim(
            cam:     *mut mjvCamera, 
            opt:     *mut mjvOption, 
            pert:    *mut mjvPerturb, 
            passive: bool
        ) -> UniquePtr<Simulate>;
        unsafe fn runPhysicsLoop(
            sim:     UniquePtr<Simulate>, 
        );
    }
}

//include_cpp! {
    //#include "../../../mujoco/simulate/simulate.h"
    //safety!(unsafe)
    //generate!("mujoco::Simulate")
//}

fn main() {
    unsafe {
        mujoco_sys::mjv_defaultScene(SCN);
        mujoco_sys::mjv_defaultCamera(CAM);
        mujoco_sys::mjv_defaultOption(OPT);
        mujoco_sys::mjv_defaultPerturb(PERT);


        let ptr = ffi::createSim(
            CAM as  *mut ffi::mjvCamera, 
            OPT as  *mut ffi::mjvOption, 
            PERT as *mut ffi::mjvPerturb, 
            false
        );

        SIM = ptr.as_mut_ptr() as *mut mujoco_Simulate;

        thread::spawn(|| {
            sleep_ms(500);
            let mut drone = drone::Drone::new((*SIM).m_, (*SIM).d_, [0.,0.,1.0]);
            let instant   = std::time::Instant::now();
            let start     = instant.elapsed().as_secs_f64();
            let length = Duration::from_secs(15);
            let mut step = 0;
            let mut completed = 0;

            while instant.elapsed() < length {
                let now  = instant.elapsed().as_secs_f64(); 
                let diff = now - start;

                if diff > 5. && completed < 1 {
                    println!("First waypoint");
                    drone.planner.borrow_mut().update_target([2.,2.,2.]);
                    completed += 1;
                }
                if (diff) > 10. && completed < 2 {
                    println!("Second waypoint");
                    drone.planner.borrow_mut().update_target([-1.,1.,3.]);
                    completed += 1;
                }
                if (diff) > 18. && completed < 3 {
                    println!("Third waypoint");
                    drone.planner.borrow_mut().update_target([-1.,-1.,0.5]);
                    completed += 1;
                }
                if step % 20 == 0 {
                    drone.outer();
                }
                drone.inner();
                //mj_step((*SIM).m_, (*SIM).d_);
                mujoco_Simulate_Sync(SIM, true);
                step += 1;

                let time   = instant.elapsed().as_secs_f64();
                let t_next = (*(*SIM).m_).opt.timestep - (time - now);
                if t_next > 0. {
                    sleep(std::time::Duration::from_secs_f64(t_next));
                }
            }
        });

        // it will clean up MODEL and DATA when it closes
        // probably should move these routines into rust!
        ffi::runPhysicsLoop(ptr);

        mujoco_sys::mjv_freeScene(SCN);
        mujoco_sys::mjr_freeContext(CON);
    }
}
