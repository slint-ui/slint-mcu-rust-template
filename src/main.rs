#![no_std]
#![cfg_attr(not(feature = "simulator"), no_main)]

extern crate alloc;

slint::include_modules!();

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });
    ui
}

#[cfg(feature = "simulator")]
fn main() -> Result<(), slint::PlatformError> {
    create_slint_app().run()
}

#[cfg(not(feature = "simulator"))]
#[rp_pico::entry]
fn main() -> ! {
    // Pull in any important traits
    use embedded_hal::digital::OutputPin;
    use panic_halt as _;
    use rp_pico::hal;
    use rp_pico::hal::pac;
    use rp_pico::hal::prelude::*;
    use slint::platform::WindowEvent;

    // -------- Setup Allocator --------
    const HEAP_SIZE: usize = 200 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    #[global_allocator]
    static ALLOCATOR: embedded_alloc::LlffHeap = embedded_alloc::LlffHeap::empty();
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    // -------- Setup peripherials --------
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let spi_sclk = pins.gpio10.into_function::<hal::gpio::FunctionSpi>();
    let spi_mosi = pins.gpio11.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio12.into_function::<hal::gpio::FunctionSpi>();

    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        shared_spi::DISPLAY_FREQ,
        embedded_hal::spi::MODE_3,
    );
    // The display and the touch screen share the SPI bus.
    let spi = cortex_m::singleton!(: shared_spi::Bus = shared_spi::Bus::new(spi)).unwrap();

    let mut bl = pins.gpio13.into_push_pull_output();
    let rst = pins.gpio15.into_push_pull_output();
    let dc = pins.gpio8.into_push_pull_output();
    let cs = pins.gpio9.into_push_pull_output();
    let display_buffer = cortex_m::singleton!(: [u8; 512] = [0; 512]).unwrap();
    let di = mipidsi::interface::SpiInterface::new(
        spi.device(cs, shared_spi::DISPLAY_FREQ),
        dc,
        display_buffer,
    );
    let mut display = mipidsi::Builder::new(mipidsi::models::ST7789, di)
        .reset_pin(rst)
        .display_size(240, 320)
        .orientation(mipidsi::options::Orientation::new().rotate(mipidsi::options::Rotation::Deg90))
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .init(&mut timer)
        .unwrap();
    bl.set_high().unwrap();

    // touch screen
    let touch_irq = pins.gpio17.into_pull_up_input();
    let mut touch_cs = pins.gpio16.into_push_pull_output();
    touch_cs.set_high().unwrap();
    let mut touch = xpt2046::XPT2046::new(touch_irq, spi.device(touch_cs, xpt2046::SPI_FREQ));

    // -------- Setup the Slint backend --------
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());
    window.set_size(slint::PhysicalSize::new(320, 240));
    slint::platform::set_platform(alloc::boxed::Box::new(MyPlatform {
        window: window.clone(),
        timer,
    }))
    .unwrap();

    struct MyPlatform {
        window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
        timer: hal::Timer,
    }

    impl slint::platform::Platform for MyPlatform {
        fn create_window_adapter(
            &self,
        ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError>
        {
            Ok(self.window.clone())
        }
        fn duration_since_start(&self) -> core::time::Duration {
            core::time::Duration::from_micros(self.timer.get_counter().ticks())
        }
    }

    // -------- Configure the UI --------
    // (need to be done after the call to slint::platform::set_platform)
    let _ui = create_slint_app();

    // -------- Event loop --------
    let mut line = [slint::platform::software_renderer::Rgb565Pixel(0); 320];
    let mut last_touch = None;
    loop {
        slint::platform::update_timers_and_animations();
        window.draw_if_needed(|renderer| {
            use embedded_graphics_core::prelude::*;
            struct DisplayWrapper<'a, T>(
                &'a mut T,
                &'a mut [slint::platform::software_renderer::Rgb565Pixel],
            );
            impl<T: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>
                slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
            {
                type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
                fn process_line(
                    &mut self,
                    line: usize,
                    range: core::ops::Range<usize>,
                    render_fn: impl FnOnce(&mut [Self::TargetPixel]),
                ) {
                    let rect = embedded_graphics_core::primitives::Rectangle::new(
                        Point::new(range.start as _, line as _),
                        Size::new(range.len() as _, 1),
                    );
                    render_fn(&mut self.1[range.clone()]);
                    // NOTE! this is not an efficient way to send pixel to the screen, but it is kept simple on this template.
                    // It would be much faster to use the DMA to send pixel in parallel.
                    // See the example in https://github.com/slint-ui/slint/blob/master/examples/mcu-board-support/pico_st7789/pico_st7789.rs
                    self.0
                        .fill_contiguous(
                            &rect,
                            self.1[range.clone()].iter().map(|p| {
                                embedded_graphics_core::pixelcolor::raw::RawU16::new(p.0).into()
                            }),
                        )
                        .map_err(drop)
                        .unwrap();
                }
            }
            renderer.render_by_line(DisplayWrapper(&mut display, &mut line));
        });

        // handle touch event
        let button = slint::platform::PointerEventButton::Left;
        if let Some(event) = touch
            .read()
            .map_err(|_| ())
            .unwrap()
            .map(|point| {
                let position =
                    slint::PhysicalPosition::new((point.0 * 320.) as _, (point.1 * 240.) as _)
                        .to_logical(window.scale_factor());
                match last_touch.replace(position) {
                    Some(_) => WindowEvent::PointerMoved { position },
                    None => WindowEvent::PointerPressed { position, button },
                }
            })
            .or_else(|| {
                last_touch.take().map(|position| WindowEvent::PointerReleased { position, button })
            })
        {
            window.dispatch_event(event);
            // Don't go to sleep after a touch event that forces a redraw
            continue;
        }

        if window.has_active_animations() {
            continue;
        }

        // TODO: we could save battery here by going to sleep up to
        //   slint::platform::duration_until_next_timer_update()
        // or until the next touch interrupt, whatever comes first
        // cortex_m::asm::wfe();
    }
}

/// Lets the display and the touch screen share the SPI bus, each with its own frequency.
#[cfg(not(feature = "simulator"))]
mod shared_spi {
    use core::cell::RefCell;
    use embedded_hal::digital::OutputPin;
    use embedded_hal::spi::{ErrorType, Operation, SpiBus, SpiDevice};
    use fugit::{Hertz, RateExtU32};
    use rp_pico::hal;

    // 16ns for serial clock cycle (write), page 43 of https://www.waveshare.com/w/upload/a/ae/ST7789_Datasheet.pdf
    pub const DISPLAY_FREQ: Hertz<u32> = Hertz::<u32>::Hz(62_500_000);

    type Pins = (
        hal::gpio::Pin<hal::gpio::bank0::Gpio11, hal::gpio::FunctionSpi, hal::gpio::PullDown>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio12, hal::gpio::FunctionSpi, hal::gpio::PullDown>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio10, hal::gpio::FunctionSpi, hal::gpio::PullDown>,
    );
    type Spi = hal::Spi<hal::spi::Enabled, hal::pac::SPI1, Pins, 8>;

    pub struct Bus(RefCell<(Spi, Hertz<u32>)>);

    impl Bus {
        pub fn new(spi: Spi) -> Self {
            Self(RefCell::new((spi, DISPLAY_FREQ)))
        }

        pub fn device<CS: OutputPin>(&self, cs: CS, freq: Hertz<u32>) -> Device<'_, CS> {
            Device { bus: self, cs, freq }
        }
    }

    pub struct Device<'a, CS> {
        bus: &'a Bus,
        cs: CS,
        freq: Hertz<u32>,
    }

    impl<CS> ErrorType for Device<'_, CS> {
        type Error = <Spi as ErrorType>::Error;
    }

    impl<CS: OutputPin> SpiDevice for Device<'_, CS> {
        fn transaction(&mut self, operations: &mut [Operation<u8>]) -> Result<(), Self::Error> {
            let mut bus = self.bus.0.borrow_mut();
            if bus.1 != self.freq {
                bus.0.flush()?;
                bus.0.set_baudrate(125_000_000u32.Hz(), self.freq);
                bus.1 = self.freq;
            }
            self.cs.set_low().ok();
            for op in operations {
                match op {
                    Operation::Read(words) => bus.0.read(words),
                    Operation::Write(words) => bus.0.write(words),
                    Operation::Transfer(read, write) => bus.0.transfer(read, write),
                    Operation::TransferInPlace(words) => bus.0.transfer_in_place(words),
                    Operation::DelayNs(_) => unimplemented!(),
                }?;
            }
            bus.0.flush()?;
            self.cs.set_high().ok();
            Ok(())
        }
    }
}

#[cfg(not(feature = "simulator"))]
mod xpt2046 {
    use embedded_hal::digital::InputPin;
    use embedded_hal::spi::SpiDevice;
    use fugit::Hertz;

    pub const SPI_FREQ: Hertz<u32> = Hertz::<u32>::Hz(3_000_000);

    pub struct XPT2046<IRQ: InputPin, SPI: SpiDevice> {
        irq: IRQ,
        spi: SPI,
        pressed: bool,
    }

    impl<IRQ: InputPin, SPI: SpiDevice> XPT2046<IRQ, SPI> {
        pub fn new(irq: IRQ, spi: SPI) -> Self {
            Self { irq, spi, pressed: false }
        }

        pub fn read(&mut self) -> Result<Option<(f32, f32)>, Error<IRQ::Error, SPI::Error>> {
            const PRESS_THRESHOLD: i32 = -25_000;
            const RELEASE_THRESHOLD: i32 = -30_000;
            let threshold = if self.pressed { RELEASE_THRESHOLD } else { PRESS_THRESHOLD };
            self.pressed = false;

            if self.irq.is_low().map_err(Error::Pin)? {
                const CMD_X_READ: u8 = 0b10010000;
                const CMD_Y_READ: u8 = 0b11010000;
                const CMD_Z1_READ: u8 = 0b10110000;
                const CMD_Z2_READ: u8 = 0b11000000;

                // These numbers were measured approximately.
                const MIN_X: u32 = 1900;
                const MAX_X: u32 = 30300;
                const MIN_Y: u32 = 2300;
                const MAX_Y: u32 = 30300;

                macro_rules! xchg {
                    ($byte:expr) => {{
                        let mut b = [0, $byte, 0, 0];
                        self.spi.transfer_in_place(&mut b).map_err(Error::Transfer)?;
                        let [_, _, h, l] = b;
                        ((h as u32) << 8) | (l as u32)
                    }};
                }

                let z1 = xchg!(CMD_Z1_READ);
                let z2 = xchg!(CMD_Z2_READ);
                let z = z1 as i32 - z2 as i32;

                if z < threshold {
                    return Ok(None);
                }

                let mut point = (0u32, 0u32);
                for _ in 0..10 {
                    let y = xchg!(CMD_Y_READ);
                    let x = xchg!(CMD_X_READ);
                    point.0 += i16::MAX as u32 - x;
                    point.1 += y;
                }

                let z1 = xchg!(CMD_Z1_READ);
                let z2 = xchg!(CMD_Z2_READ);
                let z = z1 as i32 - z2 as i32;

                if z < RELEASE_THRESHOLD {
                    return Ok(None);
                }

                point.0 /= 10;
                point.1 /= 10;
                self.pressed = true;
                Ok(Some((
                    point.0.saturating_sub(MIN_X) as f32 / (MAX_X - MIN_X) as f32,
                    point.1.saturating_sub(MIN_Y) as f32 / (MAX_Y - MIN_Y) as f32,
                )))
            } else {
                Ok(None)
            }
        }
    }

    pub enum Error<PinE, TransferE> {
        Pin(PinE),
        Transfer(TransferE),
    }
}
