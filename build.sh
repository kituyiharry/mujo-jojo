echo "============ building mujoco from source"
git clone --depth=1 https://github.com/google-deepmind/mujoco.git;
pushd $(pwd);
# this may work on Linux
#AR=$(which ar)
#RANLIB=$(which ranlib)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "please install mold linker for quicker (and saner) setup"
    cmake -DCMAKE_LINKER_TYPE=MOLD -DCMAKE_CXX_COMPILER_AR=/usr/bin/ar -DCMAKE_C_COMPILER_AR=/usr/bin/ar -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_COMPILER_RANLIB=/usr/bin/ranlib -DCMAKE_C_COMPILER_RANLIB=/usr/bin/ranlib  -DCMAKE_INSTALL_PREFIX=/usr \
        -B build \
        -G Ninja \
        -Wno-dev && cd ./mujoco/build && ninja && cd ../../
else
    cd ./mujoco &&  cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=true -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=false && cmake --build build --parallel --config=Release;
fi
popd;
CWD=$(pwd);
export BINDGEN_MUJOCO_PATH=$CWD/mujoco;
env;
echo "============ building libs"
cd ./mujoco-sys;
BINDGEN_MUJOCO_PATH=$CWD/mujoco cargo build --release && cd ..;
mkdir -p ./mujoco-sys/examples/mujo-jojo/target/release;
mkdir -p ./mujoco-sys/examples/mujo-jojo/target/debug;
mkdir -p ./mujoco-sys/examples/mujoco-sim-cpp/target/release;
mkdir -p ./mujoco-sys/examples/mujoco-sim-cpp/target/debug;
mkdir -p ./mujoco-sys/target/release;
mkdir -p ./mujoco-sys/target/debug;
pwd;
echo "============ copying libs"
`cp -rfv ./mujoco/build/lib/* ./mujoco-sys/target/release/`;
`cp -rfv ./mujoco/build/lib/* ./mujoco-sys/target/debug/`;
`cp -rfv ./mujoco/build/lib/* ./mujoco-sys/examples/mujo-jojo/target/release/`;
`cp -rfv ./mujoco/build/lib/* ./mujoco-sys/examples/mujo-jojo/target/debug/`;
`cp -rfv ./mujoco/build/lib/* ./mujoco-sys/examples/mujoco-sim-cpp/target/release/`;
`cp -rfv ./mujoco/build/lib/* ./mujoco-sys/examples/mujoco-sim-cpp/target/debug/`;
