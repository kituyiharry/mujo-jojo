## Mujoco Sys Rust binding

```bash
# run this and cross fingers (tested on macos only)
bash build.sh
```

Direct bindings that at least work on MacOS. No need to modify sources. just
download or clone into `$BINDGEN_MUJOCO_PATH` , build that project into a build
directory and then cargo build here. You may need to copy over the files
generated shared libraries in `$BINDGEN_MUJOCO_PATH` into a directory that can
be found by the linker in order to run examples.
This project may not work properly on linux - but the others might.

Also make sure glfw is available on your system for easier setup.
