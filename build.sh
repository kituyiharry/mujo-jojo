echo "============ building mujoco from source"
git clone --depth=1 https://github.com/google-deepmind/mujoco.git;
pushd $(pwd);
# this may work on Linux
#AR=$(which ar)
#RANLIB=$(which ranlib)
#cd ./mujoco &&  cmake -B build -S . -DCMAKE_CXX_COMPILER_AR=$AR -DCMAKE_C_COMPILER_AR=$AR -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_RANLIB=$RANLIB -DCMAKE_C_COMPILER_RANLIB=$RANLIB -DBUILD_SHARED_LIBS:BOOL=true -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=false && cmake --build build --parallel --target glfw libmujoco_simulate --config=Release;
cd ./mujoco &&  cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=true -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=false && cmake --build build --parallel --target glfw libmujoco_simulate --config=Release;
popd;
CWD=$(pwd);
export BINDGEN_MUJOCO_PATH=$CWD/mujoco;
env;
echo "============ building libs"
cd ./mujoco-sys;
BINDGEN_MUJOCO_PATH=$CWD/mujoco cargo build --release && cd ..;
mkdir -p ./mujoco-sys/examples/mujo-jojo/target/release;
mkdir -p ./mujoco-sys/examples/mujo-jojo/target/debug;
mkdir -p ./mujoco-sys/target/release;
mkdir -p ./mujoco-sys/target/debug;
pwd;
echo "============ copying libs"
`cp -rfv ./mujoco/build/lib/* ./mujoco-sys/target/release/`;
`cp -rfv ./mujoco/build/lib/* ./mujoco-sys/target/debug/`;
`cp -rfv ./mujoco/build/lib/* ./mujoco-sys/examples/mujo-jojo/target/release/`;
`cp -rfv ./mujoco/build/lib/* ./mujoco-sys/examples/mujo-jojo/target/debug/`;
