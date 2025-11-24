use mujoco_sys::{self, mjrContext, mjvCamera, mjvGLCamera_, mjvGeom_, mjvLight, mjvLight_, mjvOption, mjvScene};
use glow::HasContext;
use glfw_sys::*;
use std::{ffi::CStr, ptr};
use std::ffi::{CString, c_char};

// Adapted by hand from Mujoco sample and also some of glfw_sys init code.
// https://github.com/google-deepmind/mujoco/blob/main/sample/basic.cc

static mut MODEL: *mut mujoco_sys::mjModel = ptr::null_mut();
static mut DATA : *mut mujoco_sys::mjData_ = ptr::null_mut();
static mut CAM  : *mut mujoco_sys::mjvCamera = &mut mjvCamera{ 
    type_:        0, 
    fixedcamid:   0, 
    trackbodyid:  0, 
    lookat:       [0.,0.,0.], 
    distance:     0., 
    azimuth:      0., 
    elevation:    0., 
    orthographic: 0 
};                      // abstract camera
static mut OPT  : *mut mujoco_sys::mjvOption = &mut mjvOption{
    label:          0,
    frame:          0,
    geomgroup:      [0u8,0,0,0,0,0],
    sitegroup:      [0u8,0,0,0,0,0],
    jointgroup:     [0u8,0,0,0,0,0],
    tendongroup:    [0u8,0,0,0,0,0],
    actuatorgroup:  [0u8,0,0,0,0,0],
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


// mouse interaction
static mut BUTTON_LEFT  : bool = false;
static mut BUTTON_MIDDLE: bool = false;
static mut BUTTON_RIGHT : bool = false;
static mut LASTX: f64 = 0.;
static mut LASTY: f64 = 0.;

// keyboard callback
extern "C" fn keyboard(_window: *mut GLFWwindow, key: i32, _scancode: i32, act: i32, _mods: i32) {
    unsafe {
        // backspace: reset simulation
        if act==GLFW_PRESS && (key==GLFW_KEY_DELETE || key==GLFW_KEY_BACKSPACE) {
            mujoco_sys::mj_resetData(MODEL, DATA);
            mujoco_sys::mj_forward(MODEL, DATA);
        }
    }
}


// mouse button callback
extern "C" fn mouse_button(window: *mut GLFWwindow, _button: i32, _act: i32, _mods: i32) {
    unsafe  {
        // update button state
        BUTTON_LEFT   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS)   as bool;
        BUTTON_MIDDLE = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS) as bool;
        BUTTON_RIGHT  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS)  as bool;
        // update mouse position
        glfwGetCursorPos(window, &raw mut LASTX as *mut f64, &raw mut LASTY as *mut f64);
    }
}

// mouse move callback
extern "C" fn mouse_move(window: *mut GLFWwindow, xpos: f64, ypos: f64) {
    unsafe  {
        // no buttons down: nothing to do
        if !BUTTON_LEFT && !BUTTON_MIDDLE && !BUTTON_RIGHT {
            return;
        }

        // compute mouse displacement, save
        let dx = xpos - LASTX;
        let dy = ypos - LASTY;
        LASTX = xpos;
        LASTY = ypos;

        // get current window size
        let (mut width, mut height) = (0, 0);
        glfwGetWindowSize(window, &raw mut width as *mut i32, &raw mut height as *mut i32);

        // get shift key state
        let mod_shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS;

        // determine action based on mouse button
        let action: mujoco_sys::mjtMouse;
        if BUTTON_RIGHT {
            action = if mod_shift { mujoco_sys::mjtMouse::mjMOUSE_MOVE_H } else { mujoco_sys::mjtMouse::mjMOUSE_MOVE_V };
        } else if BUTTON_LEFT {
            action = if mod_shift { mujoco_sys::mjtMouse::mjMOUSE_ROTATE_H } else { mujoco_sys::mjtMouse::mjMOUSE_ROTATE_V };
        } else {
            action = mujoco_sys::mjtMouse::mjMOUSE_ZOOM;
        }

        // move camera
        mujoco_sys::mjv_moveCamera(MODEL, action as i32, dx/(height as f64), dy/(height as f64), SCN, CAM);
    }
}


// scroll callback
extern  "C" fn scroll(_window: *mut GLFWwindow, _xoffset: f64, yoffset: f64) {
  // emulate vertical mouse motion = 5% of window height
  unsafe  {
    mujoco_sys::mjv_moveCamera(MODEL, mujoco_sys::mjtMouse::mjMOUSE_ZOOM as std::ffi::c_int, 0., -0.05*yoffset, SCN, CAM);
  }
}


const EXAMPLE_MODEL: &str = r#"
<mujoco model="aerialdrone">

    <compiler angle="radian" coordinate="local"/>

    <option timestep="0.002" gravity="0 0 -9.81">
        <flag contact="enable"/>
    </option>

    <asset>
        <texture  name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
        <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
        <!-- sets the sky -->
        <texture  name="skybox" type="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="512" height="512"/>
    </asset>

    <worldbody>

        <!-- Environment -->
        <geom name="floor" type="plane" size="10 10 0.1" material="grid"/>
        <light directional="true" pos="0 0 3" dir="0 0 -1"/>


        <!-- Kinematic tree of the drone -->
        <body name="base" pos="0 0 2">

            <inertial pos="0 0 0" mass="0.5" diaginertia="0.002 0.002 0.004"/>
            <geom name="base_body" type="box" size="0.25 0.15 0.05" rgba="0.2 0.2 0.8 1"/>

            <!-- Four rotors (visual only, simplified) (no mass in this simulation??)-->
            <geom name="rotor_1" type="cylinder" size="0.08 0.01" pos=" 0.2  0.2 0.03" rgba="0.3 0.3 0.3 0.7"/>
            <geom name="rotor_2" type="cylinder" size="0.08 0.01" pos="-0.2  0.2 0.03" rgba="0.3 0.3 0.3 0.7"/>
            <geom name="rotor_3" type="cylinder" size="0.08 0.01" pos=" 0.2 -0.2 0.03" rgba="0.3 0.3 0.3 0.7"/>
            <geom name="rotor_4" type="cylinder" size="0.08 0.01" pos="-0.2 -0.2 0.03" rgba="0.3 0.3 0.3 0.7"/>

            <site name="rotor_1_site" pos=" 0.2  0.2  0.03" />
            <site name="rotor_2_site" pos="-0.2  0.2  0.03" />
            <site name="rotor_3_site" pos=" 0.2 -0.2  0.03" />
            <site name="rotor_4_site" pos="-0.2 -0.2  0.03" />

            <!-- Free joint for aerial movement - gives the drone (6 DOF) -->
            <freejoint name="base_free"/>

            <!-- IMU sensor on base -->
            <site name="imu_site" pos="0 0 0" size="0.01"/>

            <!-- RGB Camera (front-facing) -->
            <camera name="rgb_camera"  pos="0.3 0 0" quat="0.707 0 0 0.707" fovy="60"/>
            <site   name="camera_site" pos="0.3 0 0" size="0.02" rgba="0 1 0 0.5"/>

            <!-- LIDAR sensor site (top-mounted) -->
            <site name="lidar_site" pos="0 0 0.1" size="0.03" rgba="1 0 0 0.5"/>
        </body>

    </worldbody>


    <sensor>
        <!-- IMU Sensors -->
        <accelerometer name="imu_accel" site="imu_site"/>
        <gyro          name="imu_gyro"  site="imu_site"/>
        <magnetometer  name="imu_mag"   site="imu_site"/>
        <framequat     name="imu_quat"  objtype="site" objname="imu_site"/>

        <!-- Range sensors for LIDAR simulation (8 rays in horizontal plane) -->
        <rangefinder name="lidar_0"   site="lidar_site" cutoff="10"/>
        <rangefinder name="lidar_45"  site="lidar_site" cutoff="10"/>
        <rangefinder name="lidar_90"  site="lidar_site" cutoff="10"/>
        <rangefinder name="lidar_135" site="lidar_site" cutoff="10"/>
        <rangefinder name="lidar_180" site="lidar_site" cutoff="10"/>
        <rangefinder name="lidar_225" site="lidar_site" cutoff="10"/>
        <rangefinder name="lidar_270" site="lidar_site" cutoff="10"/>
        <rangefinder name="lidar_315" site="lidar_site" cutoff="10"/>
    </sensor>

    <actuator>

        <!-- Thrust actuators for aerial control (simplified) -->
        <motor name="thrust" joint="base_free" gear="0 0 1 0 0 0" ctrlrange="0 20"/>
        <motor name="roll"   joint="base_free" gear="1 0 0 0 0 0" ctrlrange="-5 5"/>
        <motor name="pitch"  joint="base_free" gear="0 1 0 0 0 0" ctrlrange="-5 5"/>
        <motor name="yaw"    joint="base_free" gear="0 0 0 0 0 1" ctrlrange="-5 5"/>

    </actuator>

</mujoco>
"#;

pub fn main() {
    unsafe  {
        let mut errbuf = Vec::with_capacity(1024);
        let p_error: *mut std::ffi::c_char = errbuf.as_mut_ptr();
        let p_errsz: std::ffi::c_int  = 0;

        let ver = mujoco_sys::mj_version();
        let mut major = 0;
        let mut minor = 0;
        let mut patch = 0;
        glfw_sys::glfwGetVersion(&mut major, &mut minor, &mut patch);
        eprintln!("Mujoco Version: {ver}");
        eprintln!("GLFW version {}.{}.{}", major, minor, patch);
        let mut description: *const i8 = std::ptr::null();

        // avoid initializing outside main because it fails for some reason
        let c_string = CString::new(EXAMPLE_MODEL).expect("String contained interior null bytes");
        let c_char_ptr = c_string.as_c_str();
        let mptr = c_char_ptr.as_ptr();

        let spec = mujoco_sys::mj_parseXMLString(mptr, ptr::null_mut(), p_error, p_errsz);
        MODEL    = mujoco_sys::mj_compile(spec, ptr::null_mut());
        DATA     = mujoco_sys::mj_makeData(MODEL);

        if MODEL.is_null() || DATA.is_null() {
            eprintln!("NO SPEC or MODEL!!");
            let rust_str = CStr::from_ptr(p_error).to_str().expect("C string contained invalid UTF-8");
            if p_error.is_null() {
                eprintln!("{rust_str} no error!!??: {p_errsz}");
            } else {
                eprintln!("{rust_str} error: {p_errsz}");
            }
            return
        }

        if glfwInit() != GLFW_TRUE {
            glfwGetError(&mut description);
            panic!(
                "Error: {:?}\n",
                description
                    .is_null()
                    .then_some(c"")
                    .unwrap_or_else(|| CStr::from_ptr(description))
            );
        }

        eprintln!("Initialized GLFW");
        let win = glfwCreateWindow(
            1280,
            720,
            c"Mujoco Basic".as_ptr(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        );

        glfwSetKeyCallback(win,         Some(keyboard));
        glfwSetMouseButtonCallback(win, Some(mouse_button));
        glfwSetCursorPosCallback(win,   Some(mouse_move));
        glfwSetScrollCallback(win,      Some(scroll));

        if win.is_null() {
            glfwGetError(&mut description);

            eprintln!(
                "Error: {:?}\n",
                description
                    .is_null()
                    .then_some(c"")
                    .unwrap_or_else(|| CStr::from_ptr(description))
            );
            glfwTerminate();
            panic!();
        }
        eprintln!("Created GLFW window with handle: {:?}", win);
        glfwMakeContextCurrent(win);
        glfwSwapInterval(1);


        // initialize visualization data structures
        mujoco_sys::mjv_defaultCamera(CAM);
        mujoco_sys::mjv_defaultOption(OPT);
        mujoco_sys::mjv_defaultScene(SCN);
        mujoco_sys::mjr_defaultContext(CON);

        // create scene and context
        mujoco_sys::mjv_makeScene(MODEL, SCN, 2000);
        mujoco_sys::mjr_makeContext(MODEL, CON, mujoco_sys::mjtFontScale::mjFONTSCALE_150 as i32);

        let ctx = glow::Context::from_loader_function_cstr(|s| {
            glfwGetProcAddress(s.as_ptr())
                .map(|p| p as _)
                .unwrap_or(std::ptr::null())
        });
        println!("Created OpenGL context with handle: {:?}\n", ctx);

        // browser will call this closure every frame
        // Try to never block inside this and always return control flow to browser.
        //set_main_loop_callback(move || {
        //'MAINLOOP: loop {
        while !(glfwWindowShouldClose(win) != 0) {
            // lets animate clear color based on time
            let time = glfwGetTime() as f32;

            // advance interactive simulation for 1/60 sec
            //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
            //  this loop will finish on time for the next frame to be rendered at 60 fps.
            //  Otherwise add a cpu timer and exit this loop when it is time to render.
            let simstart = (*DATA).time;

            while ((*DATA).time - simstart) < 1.0/60.0 {
                //eprintln!("\r-----+");
                mujoco_sys::mj_step(MODEL, DATA);
            }

            //ctx.clear_color(time.sin(), time.cos(), time.tan(), 1.0);
            //ctx.clear(glow::COLOR_BUFFER_BIT);

            let mut viewport = mujoco_sys::mjrRect_ { 
                left: 0, bottom: 0, width: 0, height: 0 
            };

            glfwGetFramebufferSize(win, &mut viewport.width as *mut i32, &mut viewport.height as *mut i32,);

            // update scene and render
            mujoco_sys::mjv_updateScene(MODEL, DATA, OPT, ptr::null_mut(), CAM, mujoco_sys::mjtCatBit_::mjCAT_ALL as i32, SCN);
            mujoco_sys::mjr_render(viewport, SCN, CON);

            glfwSwapBuffers(win);
            // don't use waitEvents as that might block event-loop inside wasm
            // and never return control flow to browser
            glfwPollEvents();

            if glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_TRUE {
                glfwMakeContextCurrent(std::ptr::null_mut());
                glfwDestroyWindow(win);
                //break;
                //emscripten_cancel_main_loop();
            }
        }
        //});

        //free visualization storage
        mujoco_sys::mjv_freeScene(SCN);
        mujoco_sys::mjr_freeContext(CON);

        // free MuJoCo model and data
        mujoco_sys::mj_deleteData(DATA);
        mujoco_sys::mj_deleteModel(MODEL);

        if cfg!(not(linux)) {
            glfwTerminate();
        }
    }

}

// from glfw_sys basic example -> used for wasm target

// #[allow(non_camel_case_types)]
// type em_callback_func = unsafe extern "C" fn();

// #[allow(unused)]
//const CANVAS_ELEMENT_NAME: *const std::ffi::c_char = "#canvas\0".as_ptr() as _;
//extern "C" {
//pub fn emscripten_cancel_main_loop();
//pub fn emscripten_set_main_loop(
//func: em_callback_func,
//fps: std::ffi::c_int,
//simulate_infinite_loop: std::ffi::c_int,
//);
//}

//thread_local!(static MAIN_LOOP_CALLBACK: std::cell::RefCell<Option<Box<dyn FnMut()>>>  = std::cell::RefCell::new(None));

//pub fn set_main_loop_callback<F: 'static>(callback: F)
//where
    //F: FnMut(),
//{
    //MAIN_LOOP_CALLBACK.with(|log| {
        //*log.borrow_mut() = Some(Box::new(callback));
    //});

    ////unsafe {
        //////emscripten_set_main_loop(wrapper::<F>, 0, 1);
    ////}
    //#[allow(clippy::extra_unused_type_parameters)]
    //extern "C" fn wrapper<F>()
//where
        //F: FnMut(),
    //{
        //MAIN_LOOP_CALLBACK.with(|z| {
            //if let Some(ref mut callback) = *z.borrow_mut() {
                //callback();
            //}
        //});
    //}
//}
