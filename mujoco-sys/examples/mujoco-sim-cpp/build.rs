use std::{env, path::PathBuf};

fn main() {

    println!("cargo:rerun-if-env-changed=BINDGEN_MUJOCO_PATH");
    println!("Ensure that BINDGEN_MUJOCO_PATH is set to get headers and xcrun from xcode command line tools is available!");

    let mujoco_path = if let Ok(path) = env::var("BINDGEN_MUJOCO_PATH") {
        PathBuf::from(path)
    } else {
        panic!("BINDGEN_MUJOCO_PATH not found in env");
    };

    let mujoco_simulate_path= PathBuf::from(
        format!("{}/simulate/", mujoco_path.display())
    );
    let mujoco_lib_path= PathBuf::from(
        format!("{}/build/lib", mujoco_path.display())
    );

    println!("cargo:rerun-if-changed={}",mujoco_simulate_path.join("simulate.cc").display());
    println!("cargo:rerun-if-changed={}",mujoco_simulate_path.join("simulate.h").display());

    println!("cargo:rustc-link-search={}", mujoco_simulate_path.canonicalize().unwrap().display());
    println!("cargo:rustc-link-search={}", mujoco_lib_path.canonicalize().unwrap().display());

    println!("cargo:rerun-if-changed=src/main.rs");

    println!("cargo:rustc-link-search={}", mujoco_lib_path.canonicalize().unwrap().display());

    println!("cargo:rustc-link-lib=mujoco");
    println!("cargo:rustc-link-lib=dylib=glfw3");
    println!("cargo:rustc-link-lib=lodepng");
    println!("cargo:rustc-link-lib=simulate");
    println!("cargo:rustc-link-lib=tinyxml2");
    println!("cargo:rustc-link-lib=qhullstatic_r");
    println!("cargo:rustc-link-lib=ccd");

    if cfg!(target_os = "macos") {
        // macos specific
        println!("cargo:rustc-link-lib=framework=OpenGL");
        println!("cargo:rustc-link-lib=framework=QuartzCore");
        println!("cargo:rustc-link-lib=framework=Cocoa");
        println!("cargo:rustc-link-lib=framework=IOKit");
    }

    cxx_build::bridge("src/main.rs")
        .include("./", )
        .include(mujoco_simulate_path.clone())
        .include(format!("{}/include/", mujoco_path.display()))
        .include(format!("{}/build/_deps/lodepng-src/", mujoco_path.display()))
        .include(format!("{}/build/_deps/glfw3-src/include", mujoco_path.display()))
        .warnings(false)
        .file(mujoco_simulate_path.join("simulate.cc"))
        .file(mujoco_simulate_path.join("platform_ui_adapter.cc"))
        .file(mujoco_simulate_path.join("glfw_adapter.cc"))
        .file(mujoco_simulate_path.join("glfw_dispatch.cc"))
        .file("bridge.cc")
        .flags(["-x", "c++"])
        .warnings(false)
        .extra_warnings(false)
        .warnings_into_errors(false)
        .std("c++20")
        .compile("mujoco-sim-cpp")
    ;

}
