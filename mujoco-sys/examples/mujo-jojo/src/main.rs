#![allow(static_mut_refs)]
use image::{ImageBuffer, Rgb};
use foxglove::schemas::{ArrowPrimitive, Color, CompressedImage, CubePrimitive, Duration, Pose, Quaternion, SceneEntity, SceneUpdate, Timestamp, Vector3};
use foxglove::{LazyChannel, LazyRawChannel};
use mujoco_sys::{self, mjrContext, mjvCamera, mjvGLCamera_, mjvGeom_, mjvLight_, mjvOption, mjvScene};
use glfw_sys::*;
use once_cell::unsync::Lazy;
use std::cell::UnsafeCell;
use std::fs::{OpenOptions};
use std::io::{Cursor};
use std::path::Path;
use std::sync::{self};
use std::thread::{self, sleep, sleep_ms};
use std::{ffi::CStr, ptr};
use std::ffi::{CString};

use crate::drone::DroneCtx;
use crate::record::VideoEncoder;

mod pid;
mod planner;
mod drone;
mod record;

// Adapted by hand from Mujoco sample and also some of glfw_sys init code.
// https://github.com/google-deepmind/mujoco/blob/main/sample/basic.cc

static mut MODEL: *mut mujoco_sys::mjModel = ptr::null_mut();
static mut DATA : *mut mujoco_sys::mjData_ = ptr::null_mut();
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
        glfwGetCursorPos(window, &raw mut LASTX, &raw mut LASTY);
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
        glfwGetWindowSize(window, &raw mut width, &raw mut height);

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

static DRONEDATA:  LazyRawChannel = LazyRawChannel::new("/dronectx", "json");
static DRONESCENE: LazyChannel<SceneUpdate> = LazyChannel::new("/dronepose");
static IMAGECHANL: LazyChannel<CompressedImage> = LazyChannel::new("/dronecam");
//static DEPTHCHANL: LazyChannel<PointCloud> = LazyChannel::new("/dronecam");

const WIDTH : i32 = 640;
const HEIGHT: i32 = 480;
const OFFSCR: f64 = 0.5;

const CW: usize = (WIDTH  as f64*OFFSCR) as usize;
const CH: usize = (HEIGHT as f64*OFFSCR) as usize; 

static mut SRGB8: Lazy<UnsafeCell<Vec<u8>>> = Lazy::new(||{ 
    UnsafeCell::new(
        vec![0;CW*CH*3]
    )
});
static mut CNVIMG: Lazy<UnsafeCell<Vec<u8>>> = Lazy::new(||{ 
    UnsafeCell::new(
        Vec::with_capacity(CW*CH)
    )
});
static mut DEPTH: Lazy<UnsafeCell<Vec<f32>>> =Lazy::new(||{
    UnsafeCell::new(
        vec![0.;CW*CH]
    )
});

fn main() {
    let env = env_logger::Env::default().default_filter_or("debug");
    env_logger::init_from_env(env);
    unsafe {
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
        let c_str = CString::new("./aerialdrone.xml").expect("String contained interior null bytes");
        let mptr = c_str.as_c_str().as_ptr();

        //mujoco_sys::mujoco_Simulate_Load(this, m, d, displayed_filename);

        MODEL = mujoco_sys::mj_loadXML(mptr, ptr::null_mut(), p_error, p_errsz);
        DATA  = mujoco_sys::mj_makeData(MODEL);

        if MODEL.is_null() || DATA.is_null() {
            eprintln!("NO SPEC or MODEL!!");
            let rstr = CStr::from_ptr(p_error).to_str().expect("C string contained invalid UTF-8");
            if p_error.is_null() {
                eprintln!("{rstr} no error!!??: {p_errsz}");
            } else {
                eprintln!("{rstr} error: {p_errsz}");
            }
            return
        }

        if glfwInit() != GLFW_TRUE {
            glfwGetError(&mut description);
            panic!(
            "Error: {:?}\n",
            if description
            .is_null() { c"" } else { CStr::from_ptr(description) }); 
        }

        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

        eprintln!("Initialized GLFW");
        let win = glfwCreateWindow(
            WIDTH,
            HEIGHT,
            c"Mujo-Jojo Simulation".as_ptr(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        );
        // prevent resize for the sake of the camera!

        glfwSetKeyCallback(win,         Some(keyboard));
        glfwSetMouseButtonCallback(win, Some(mouse_button));
        glfwSetCursorPosCallback(win,   Some(mouse_move));
        glfwSetScrollCallback(win,      Some(scroll));

        if win.is_null() {
            glfwGetError(&mut description);

            eprintln!(
            "Error: {:?}\n",
            if description
            .is_null() { c"" } else { CStr::from_ptr(description) }
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

        // Model init
        let mut drone = drone::Drone::new(MODEL, DATA, [0.,0.,0.05]);
        let instant   = std::time::Instant::now();
        let start     = instant.elapsed().as_secs_f64();
        let mut step  = 1; 
        let mut completed = 0;

        let (netsender, netrcver) = sync::mpsc::channel::<DroneCtx>();
        let (encsender, encrcver) = sync::mpsc::channel::<usize>();

        thread::spawn(move || {
            let mut num = 0;
            let length = std::time::Duration::from_secs_f64(3.0);
            while encrcver.recv().is_ok()  {

                let now = std::time::Instant::now();
                let mut vid = VideoEncoder::new(CW, CH, 30).unwrap();

                // 2.5sec video
                println!("encoding maneouver");
                while now.elapsed() < length  {
                    let data = SRGB8.get().as_mut().unwrap();
                    let clone = data.clone();
                    let img_buffer: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::from_raw(CW as u32, CH as u32, clone).unwrap();
                    vid.add_frame(img_buffer).unwrap();
                }
                thread::spawn(move ||{
                    // This encoder is really slow on Mac!!!!!!
                    vid.finish(Path::new(&(format!("./video/maneouver_{}.ivf", num)))).unwrap();
                });
                println!("done encoding maneouver");

                num += 1;
            }
        });

        thread::spawn(move || {
            foxglove::WebSocketServer::new().bind("0.0.0.0", 8765)
                .start_blocking()
                .expect("Server failed to start");
            while let Ok(drctx) = netrcver.recv() {
                DRONESCENE.log(&SceneUpdate { 
                    deletions: vec![], 
                    entities:  vec![
                        SceneEntity {
                            frame_id: "skydio drone".to_owned(),
                            id: "skydio".to_owned(),
                            lifetime: Some(Duration::new(0,0)),
                            arrows: vec![
                                ArrowPrimitive { 
                                    pose: Some(Pose {
                                        position: Some(Vector3 {
                                            x: drctx.position[0],
                                            y: drctx.position[1],
                                            z: drctx.position[2],
                                        }),
                                        orientation: Some(Quaternion {
                                            x: drctx.position[3],
                                            y: drctx.position[4],
                                            z: drctx.position[5],
                                            w: drctx.position[6],
                                        }),
                                    }),
                                    shaft_length: drctx.pitch, 
                                    shaft_diameter:  0.1, 
                                    head_length:    0.01,
                                    head_diameter:  0.01, 
                                    color: Some(Color {
                                        r: 0.0,
                                        g: 1.0,
                                        b: 0.0,
                                        a: 1.0,
                                    }), 
                                },
                                ArrowPrimitive { 
                                    pose: Some(Pose {
                                        position: Some(Vector3 {
                                            x: drctx.position[0],
                                            y: drctx.position[1],
                                            z: drctx.position[2],
                                        }),
                                        orientation: Some(Quaternion {
                                            x: drctx.position[3],
                                            y: drctx.position[4],
                                            z: drctx.position[5],
                                            w: drctx.position[6],
                                        }),
                                    }),
                                    shaft_length: drctx.roll, 
                                    shaft_diameter: 0.1, 
                                    head_length:    0.01,
                                    head_diameter:  0.01, 
                                    color: Some(Color {
                                        r: 0.6,
                                        g: 0.5,
                                        b: 0.4,
                                        a: 1.0,
                                    }), 
                                },
                                ArrowPrimitive { 
                                    pose: Some(Pose {
                                        position: Some(Vector3 {
                                            x: drctx.position[0],
                                            y: drctx.position[1],
                                            z: drctx.position[2],
                                        }),
                                        orientation: Some(Quaternion {
                                            x: drctx.position[3],
                                            y: drctx.position[4],
                                            z: drctx.position[5],
                                            w: drctx.position[6],
                                        }),
                                    }),
                                    shaft_length: drctx.thrust, 
                                    shaft_diameter: 0.01, 
                                    head_length:    0.001,
                                    head_diameter:  0.001, 
                                    color: Some(Color {
                                        r: 0.0,
                                        g: 1.0,
                                        b: 1.0,
                                        a: 1.0,
                                    }), 
                                },
                            ],
                            cubes: vec![CubePrimitive {
                                pose: Some(Pose {
                                    position: Some(Vector3 {
                                        x: drctx.position[0],
                                        y: drctx.position[1],
                                        z: drctx.position[2],
                                    }),
                                    orientation: Some(Quaternion {
                                        x: drctx.position[3],
                                        y: drctx.position[4],
                                        z: drctx.position[5],
                                        w: drctx.position[6],
                                    }),
                                }),
                                size: Some(Vector3 {
                                    x: 0.16,
                                    y: 0.04,
                                    z: 0.02,
                                }),
                                color: Some(Color {
                                    r: 1.0,
                                    g: 0.0,
                                    b: 0.0,
                                    a: 1.0,
                                }),
                            }],
                            ..Default::default()
                        }
                    ]
                });
                let data = SRGB8.get().as_mut().unwrap();
                let mut ibuf = Cursor::new(CNVIMG.get().as_mut().unwrap());
                let img_buffer: Option<ImageBuffer<Rgb<u8>, &[u8]>> = ImageBuffer::from_raw(CW as u32, CH as u32, &data[..]);
                if let Some(img) = img_buffer {
                    //img.save(format!("./output_image{stepc}.png")).expect("Failed to save image");
                    img.write_to(&mut ibuf, image::ImageFormat::WebP).expect("write webp to buffer!");
                    IMAGECHANL.log(&CompressedImage { 
                        timestamp: Some(Timestamp::now()), 
                        frame_id: "stream".to_owned(), 
                        format:   "webp".into(), 
                        data:     foxglove::bytes::Bytes::copy_from_slice(CNVIMG.get().as_mut().unwrap())
                    });
                };
                match serde_json::to_string(&drctx) {
                    Ok(sjson) => {
                        DRONEDATA.log(sjson.as_bytes());
                    }
                    Err(e) => {
                        eprintln!("Error: {:?}",e.to_string())
                    }
                };
            }
        });

        drone.set_callback(Box::new(move |drctx|{
            netsender.send(drctx).unwrap();
        }));

        // These strings scare me!! sheesh
        let sim = CString::new("stream").expect("cam id name"); 
        let sim_camera = sim.as_c_str().as_ptr();
        let camid = mujoco_sys::mj_name2id(drone.model, mujoco_sys::mjtObj::mjOBJ_CAMERA as i32, sim_camera);
        if camid == -1 {
            panic!("failed to get simulation camera! bailing!");
        }
        let offscreencam = &mut mujoco_sys::mjvCamera{
            type_:      mujoco_sys::mjtCamera::mjCAMERA_FIXED as i32,
            fixedcamid: camid,
            trackbodyid: camid,
            lookat:     [0.;3],
            distance:   0.,
            azimuth:    0.,
            elevation:   0.,
            orthographic: 0,
        } as *mut mujoco_sys::mjvCamera;
        // 3 RGB channels
        
        sleep_ms(1000);

        while glfwWindowShouldClose(win) == 0 {
            // let now = glfwGetTime();

            let now  = instant.elapsed().as_secs_f64(); 
            let diff = now - start;

            if diff > 5. && completed < 1 {
                println!("First waypoint");
                encsender.send(completed).unwrap();
                drone.planner.borrow_mut().update_target([2.,2.,2.]);
                completed += 1;
            }
            if (diff) > 10. && completed < 2 {
                println!("Second waypoint");
                encsender.send(completed).unwrap();
                drone.planner.borrow_mut().update_target([-1.,1.,3.]);
                completed += 1;
            }
            if (diff) > 18. && completed < 3 {
                println!("Third waypoint");
                encsender.send(completed).unwrap();
                drone.planner.borrow_mut().update_target([-1.,-1.,0.5]);
                completed += 1;
            }

            if step % 20 == 0 {
                drone.outer();
            }
            drone.inner();
            step += 1;

            mujoco_sys::mj_step(MODEL, DATA);

            let mut viewport = mujoco_sys::mjrRect_ { left: 0, bottom: 0, width: 0, height: 0 };
            glfwGetFramebufferSize(win, &mut viewport.width, &mut viewport.height);

            let inset_width  = CW as i32;
            let inset_height = CH as i32;
            let loc_x = (viewport.width ) - inset_width;
            let loc_y = (viewport.height) - inset_height;

            let offscreenvport =  mujoco_sys::mjrRect_{ 
                left:   loc_x        ,
                bottom: loc_y        ,
                width:  inset_width  ,
                height: inset_height ,
            };

            // update scene and render
            mujoco_sys::mjv_updateScene(MODEL, DATA, OPT, ptr::null_mut(), CAM, mujoco_sys::mjtCatBit_::mjCAT_ALL as i32, SCN);
            mujoco_sys::mjr_render(viewport, SCN, CON);
            mujoco_sys::mjv_updateScene(MODEL, DATA, OPT, ptr::null_mut(), offscreencam, mujoco_sys::mjtCatBit_::mjCAT_ALL as i32, SCN);
            mujoco_sys::mjr_render(offscreenvport, SCN, CON);
            // copy pixels from image
            // WARNING: i think resizing may affect this because mujoco camera sensors are just pixels from the gpu 
            mujoco_sys::mjr_readPixels(
                SRGB8.get().as_mut().unwrap().as_mut_ptr(), 
                DEPTH.get().as_mut().unwrap().as_mut_ptr(), 
                offscreenvport, CON
            );

            glfwSwapBuffers(win);
            glfwPollEvents();

            if glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_TRUE {
                glfwMakeContextCurrent(std::ptr::null_mut());
                glfwDestroyWindow(win);
            }

            let time   = instant.elapsed().as_secs_f64();
            let t_next = (*MODEL).opt.timestep - (time - now);
            if t_next > 0. {
                sleep(std::time::Duration::from_secs_f64(t_next));
            }
        }

        //free visualization storage
        mujoco_sys::mjv_freeScene(SCN);
        mujoco_sys::mjr_freeContext(CON);

        // free MuJoCo model and data
        mujoco_sys::mj_deleteData(DATA);
        mujoco_sys::mj_deleteModel(MODEL);

        if cfg!(not(target_os = "linux")) {
            glfwTerminate();
        }
    }
}
