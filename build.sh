echo "============ building mujoco from source"
git clone --depth=1 https://github.com/google-deepmind/mujoco.git;
pushd $(pwd);
cd ./mujoco &&  cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=true -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=false && cmake --build build --parallel --target glfw libmujoco_simulate --config=Release;
popd;
export BINDGEN_MUJOCO_PATH=$(pwd)/mujoco;
echo "============ building libs"
cd ./mujoco-sys && cargo build --release && cd ..;
mkdir -p ./mujoco-sys/examples/mujo-jojo/target/release;
mkdir -p ./mujoco-sys/examples/mujo-jojo/target/debug;
mkdir -p ./mujoco-sys/target/release;
mkdir -p ./mujoco-sys/target/debug;
pwd;
echo "============ copying libs"
cp -rv ./mujoco/build/lib/. ./mujoco-sys/target/release/;
cp -rv ./mujoco/build/lib/. ./mujoco-sys/target/debug/;
cp -rv ./mujoco/build/lib/. ./mujoco-sys/examples/mujo-jojo/target/release/;
cp -rv ./mujoco/build/lib/. ./mujoco-sys/examples/mujo-jojo/target/debug/;
