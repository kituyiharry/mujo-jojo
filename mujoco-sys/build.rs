use std::env;
use std::path::PathBuf;

// Adapted (by hand!!) from:  https://github.com/davidhozic/mujoco-rs/blob/main/build.rs

use bindgen::callbacks::{ParseCallbacks, DeriveInfo};

#[derive(Debug)]
struct CloneCallback;

impl ParseCallbacks for CloneCallback {
    fn add_derives(&self, info: &DeriveInfo) -> Vec<String> {
        let mut data = vec!["Clone".into()];
        if info.name.starts_with("mjui") || info.name == "mjrRect_" {
            data.push("Copy".into());
        }

        else if info.name.starts_with("mjt") {  // enums
            data.push("Copy".into());
        }

        else if info.name.starts_with("mjv") {  
            data.push("Copy".into());
        }

        data
    }

    fn field_visibility(
        &self,
        _info: bindgen::callbacks::FieldInfo<'_>,
    ) -> Option<bindgen::FieldVisibilityKind> {
        if _info.type_name.starts_with("mjs") {
            Some(bindgen::FieldVisibilityKind::PublicCrate)
        } else { None }
    }
}


fn main() {
    let mut bindings_builder = bindgen::Builder::default();

    println!("cargo:rerun-if-env-changed=BINDGEN_MUJOCO_PATH");
    println!("Ensure that BINDGEN_MUJOCO_PATH is set to get headers and xcrun from xcode command line tools is available!");

    let mujoco_path = if let Ok(path) = env::var("BINDGEN_MUJOCO_PATH") {
        PathBuf::from(path)
    } else {
        panic!("BINDGEN_MUJOCO_PATH not found in env");
    };

    let mujoco_lib_path= PathBuf::from(
        format!("{}/build/lib", mujoco_path.display())
    );

    #[cfg(target_os = "macos")]
    let mj_lib_mujoco_path = mujoco_lib_path.join("libmujoco.dylib");

    #[cfg(target_os = "linux")]
    let mj_lib_mujoco_path = mujoco_lib_path.join("libmujoco.so");

    #[cfg(windows)]
    let mj_lib_mujoco_path = mj_lib_pathbuf.join("mujoco.lib");

    println!("cargo::rerun-if-changed={}", mj_lib_mujoco_path.canonicalize().unwrap().display());
    println!("cargo:rustc-link-search={}", mujoco_lib_path.canonicalize().unwrap().display());

    println!("cargo:rustc-link-lib=mujoco");
    println!("cargo:rustc-link-lib=dylib=glfw3");
    println!("cargo:rustc-link-lib=lodepng");
    println!("cargo:rustc-link-lib=tinyxml2");
    println!("cargo:rustc-link-lib=qhullstatic_r");
    println!("cargo:rustc-link-lib=ccd");

    if cfg!(target_os="macos") {
        println!("cargo:rustc-link-lib=framework=OpenGL");
        println!("cargo:rustc-link-lib=framework=QuartzCore");
    }

    if cfg!(target_os="linux") {
        println!("cargo:rustc-link-lib=stdc++");
    }

    bindings_builder = bindings_builder
        .clang_arg(format!("-I{}/include/", mujoco_path.display()))
        .clang_arg(format!("-I{}/include/mujoco/", mujoco_path.display()))
        .clang_arg(format!("-I{}/build/_deps/glfw3-src/include", mujoco_path.display()))
         // Add include path for system frameworks on macOS
        .header(format!("{}/include/mujoco/mujoco.h", mujoco_path.display()))
        .header(format!("{}/simulate/simulate.h", mujoco_path.display()))
        .header(format!("{}/simulate/glfw_dispatch.h", mujoco_path.display()))
        .header(format!("{}/simulate/glfw_adapter.h", mujoco_path.display()))
        //.header(format!("{}/build/_deps/glfw3-src/include/GLFW/glfw3.h", mujoco_path.display()))
        //.header(format!("{}/build/_deps/glfw3-src/include/GLFW/glfw3native.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjexport.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjdata.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjmacro.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjmodel.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjplugin.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjrender.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjsan.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjthread.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjtnum.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjui.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjvisualize.h", mujoco_path.display()))
        .header(format!("{}/include/mujoco/mjxmacro.h", mujoco_path.display()))
    ;

    let bindings = if cfg!(target_os = "macos") {

        bindings_builder.clang_arg("-isysroot")
            .clang_arg(
                String::from_utf8_lossy(&std::process::Command::new("xcrun")
                    .arg("--sdk")
                    .arg("macosx")
                    .arg("--show-sdk-path")
                    .output()
                    .expect("Failed to execute xcrun")
                    .stdout
                ).trim()
                //.trim()
            ).use_core()
            .clang_arg("-std=c++20")
            // required for macOS LLVM 8 to pick up C++ headers:
            .clang_args(&["-x", "c++"])

            .layout_tests(false)
            .derive_default(false)
            .derive_copy(false)
            .rustified_enum(".*")
            .parse_callbacks(Box::new(CloneCallback))

            .blocklist_item("std::tuple.*")
            .allowlist_item("mj.*")
            .allowlist_item("mujoco::.*")
            .allowlist_function("mujoco_cSimulate.*")
            .opaque_type("std::.*")
            .generate()
            .expect("Unable to generate bindings")

    } else {

        bindings_builder
            //.generate_inline_functions(true)
            //.default_enum_style(EnumVariation::Rust { non_exhaustive: false, })
            .use_core()
            //.clang_arg("-std=c++20")
            .layout_tests(false)
            .derive_default(false)
            .derive_copy(false)
            .rustified_enum(".*")
            .parse_callbacks(Box::new(CloneCallback))

            .blocklist_item("std::tuple.*")
            .allowlist_item("mj.*")
            .allowlist_item("mujoco::.*")
            .allowlist_function("mujoco_cSimulate.*")
            .opaque_type("std::.*")
            .generate()
            .expect("Unable to generate bindings")
    };

    // Where the bindings are generated!
    let mut fdata = bindings.to_string();
    /* Extra adjustments */
    fdata = fdata.replace("pub __lx: std_basic_string_value_type<_CharT>,", "pub __lx: std::mem::ManuallyDrop<std_basic_string_value_type<_CharT>>,");
    // Remove extra Clone
    let mut re = regex::Regex::new(r"#\[derive\((.*?Clone.*?), Clone,?(.*?)\)\]").unwrap();
    fdata = re.replace_all(&fdata, "#[derive($1, $2)]").to_string();

    // Make mjtSameFrame be MjtByte as used in all fields.
    re = regex::Regex::new(r"#\[repr\(u32\)\]\n(.*\npub enum mjtSameFrame_)").unwrap();
    fdata = re.replace(&fdata, "#[repr(u8)]\n$1").to_string();

    std::fs::write("./src/bindings.rs", fdata).unwrap();

    //bindings
    //.write_to_file("./src/bindings.rs")
    //.expect("Couldn't write bindings!");

    //println!("cargo:rustc-link-search=native={}", out_dir);
    //println!("cargo:rustc-link-lib=static=hello");
}
