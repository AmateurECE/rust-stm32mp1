This is a C++ implementation of the same Rust program in the parent directory,
which I used for reference while developing the Rust version.

# Building

```
$ conan config install https://static.ethantwardy.com/conan/config.zip
$ curl -LO https://github.com/ARM-software/LLVM-embedded-toolchain-for-Arm/releases/download/release-18.1.3/LLVM-ET-Arm-18.1.3-Linux-AArch64.tar.xz
$ CC=clang CXX=clang++ conan install -of=builddir -pr:h=armv7e-m-llvm -pr:b=default .
$ cmake --preset conan-release
$ (cd builddir && ninja)
```
