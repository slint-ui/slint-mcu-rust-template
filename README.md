# Slint Bare Metal Microcontroller Rust Template

A template for a Rust Microcontroller(MCU) application that's using [Slint](https://slint-ui.com) for the user interface.

## About

This template helps you get started developing a bare metal MCU Rust application with Slint as toolkit for the user interface.
It shows how to implement the `slint::platform::Platform` trait, and displays a simple `.slint` design on the screen.

For a template about using Slint with an operating system (Desktop, or Embedded Linux), check out the
classic template at https://github.com/slint-ui/slint-rust-template.

## Usage

1. Install [`cargo-generate`](https://github.com/cargo-generate/cargo-generate)
    ```
    cargo install cargo-generate
    ```
2. Set up a sample project with this template
    ```
    cargo generate --git https://github.com/slint-ui/slint-mcu-rust-template --name my-project
    cd my-project
    ```
3. Run on the Desktop (Simulator)
    ```
    cargo run --features simulator
    ```
4. If you have a [RaspberryPi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) with a [2.8 inch Waveshare Touch Screen](https://www.waveshare.com/pico-restouch-lcd-2.8.htm), run on the device with
    ```
    cargo build --target=thumbv6m-none-eabi --features=pico --release && elf2uf2-rs -d target/thumbv6m-none-eabi/release/project-name
    ```

In order to port to your device, you will have to replace all the code that is specific to the RaspberryPi Pico.
See also the instructions on https://slint-ui.com/snapshots/master/docs/rust/slint/docs/mcu/index.html

## Next Steps

We hope that this template helps you get started and you enjoy exploring making user interfaces with Slint. To learn more
about the Slint APIs and the `.slint` markup language check out our [online documentation](https://slint-ui.com/docs/rust/slint/).
