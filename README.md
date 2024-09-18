# Experiments with Rust on STM32MP1

The `blinky` crate provides a simple application, written in Rust, running
under ThreadX. The main thread toggles the LED every 5 seconds, and pressing
the USER1 button also toggles the LED. To build the application:

```
# Must have an ARM C compiler available to compile ThreadX
curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-aarch64-arm-none-eabi.tar.xz
tar xvf arm-gnu-toolchain-13.3.rel1-aarch64-arm-none-eabi.tar.xz

# Compile the application
(cd blinky && cargo build)
```

This application only supports the STM32MP157D-DK2 development kit running in
Engineering Mode at the moment. Toggle the boot switches to enable Engineering
Mode (see Table 5 in UM2637 for more information). Plug the board's ST-Link
connector into your Linux host, and connect to it using OpenOCD and the
provided configuration:

```
openocd -f openocd.cfg
```

Launch GDB, which is automatically configured to load the binary on startup:

```
cd blinky
gdb ./target/thumbv7em-none-eabi/debug/blinky
```
