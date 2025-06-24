# `hackrf-full`

This is a complete reimplementation of [`libhackrf`][gsg-hackrf] in Rust. It 
reproduces *all* functions from the original library, and uses [`nusb`][nusb] as 
the USB backend library. It is also completely asynchronous for every USB 
operation, including every control change.

[gsg-hackrf]: https://github.com/greatscottgadgets/hackrf
[nusb]: https://crates.io/crates/nusb
