#![no_std]
#![cfg_attr(not(feature = "simulator"), no_main)]

extern crate alloc;

slint::include_modules!();

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new();

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });
    ui
}

#[cfg(feature = "simulator")]
fn main() {
    create_slint_app().run();
}

#[cfg(not(feature = "simulator"))]
#[rp_pico::entry]
fn main() -> ! {
    // Pull in any important traits
    use fugit::RateExtU32;
    use panic_halt as _;
    use rp_pico::hal;
    use rp_pico::hal::pac;
    use rp_pico::hal::prelude::*;
    use slint::platform::WindowEvent;

    // -------- Setup Allocator --------
    const HEAP_SIZE: usize = 200 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    #[global_allocator]
    static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();
    unsafe { ALLOCATOR.init(&mut HEAP as *const u8 as usize, core::mem::size_of_val(&HEAP)) };

    // -------- Setup peripherials --------
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().raw());

    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();

    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI1);
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        62_500_000.Hz(),
        &embedded_hal::spi::MODE_3,
    );
    let spi = shared_bus::BusManagerSimple::new(spi);

    let bl = pins.gpio13.into_push_pull_output();
    let rst = pins.gpio15.into_push_pull_output();
    let dc = pins.gpio8.into_push_pull_output();
    let cs = pins.gpio9.into_push_pull_output();
    let di = display_interface_spi::SPIInterface::new(spi.acquire_spi(), dc, cs);
    let mut display = st7789::ST7789::new(di, Some(rst), Some(bl), 320, 240);

    display.init(&mut delay).unwrap();
    display.set_orientation(st7789::Orientation::Landscape).unwrap();

    // touch screen
    let touch_irq = pins.gpio17.into_pull_up_input();
    let mut touch =
        xpt2046::XPT2046::new(touch_irq, pins.gpio16.into_push_pull_output(), spi.acquire_spi())
            .unwrap();

    // -------- Setup the Slint backend --------
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new();
    slint::platform::set_platform(alloc::boxed::Box::new(MyPlatform {
        window: window.clone(),
        timer,
    }))
    .unwrap();

    struct MyPlatform {
        window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow<1>>,
        timer: hal::Timer,
    }

    impl slint::platform::Platform for MyPlatform {
        fn create_window_adapter(&self) -> alloc::rc::Rc<dyn slint::platform::WindowAdapter> {
            self.window.clone()
        }
        fn duration_since_start(&self) -> core::time::Duration {
            core::time::Duration::from_micros(self.timer.get_counter())
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

#[cfg(not(feature = "simulator"))]
mod xpt2046 {
    use embedded_hal::blocking::spi::Transfer;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use fugit::RateExtU32;

    pub struct XPT2046<IRQ: InputPin + 'static, CS: OutputPin, SPI: Transfer<u8>> {
        irq: IRQ,
        cs: CS,
        spi: SPI,
        pressed: bool,
    }

    impl<PinE, IRQ: InputPin<Error = PinE>, CS: OutputPin<Error = PinE>, SPI: Transfer<u8>>
        XPT2046<IRQ, CS, SPI>
    {
        pub fn new(irq: IRQ, mut cs: CS, spi: SPI) -> Result<Self, PinE> {
            cs.set_high()?;
            Ok(Self { irq, cs, spi, pressed: false })
        }

        pub fn read(&mut self) -> Result<Option<(f32, f32)>, Error<PinE, SPI::Error>> {
            const PRESS_THRESHOLD: i32 = -25_000;
            const RELEASE_THRESHOLD: i32 = -30_000;
            let threshold = if self.pressed { RELEASE_THRESHOLD } else { PRESS_THRESHOLD };
            self.pressed = false;

            if self.irq.is_low().map_err(|e| Error::Pin(e))? {
                const CMD_X_READ: u8 = 0b10010000;
                const CMD_Y_READ: u8 = 0b11010000;
                const CMD_Z1_READ: u8 = 0b10110000;
                const CMD_Z2_READ: u8 = 0b11000000;

                // These numbers were measured approximately.
                const MIN_X: u32 = 1900;
                const MAX_X: u32 = 30300;
                const MIN_Y: u32 = 2300;
                const MAX_Y: u32 = 30300;

                // FIXME! how else set the frequency to this device
                unsafe { set_spi_freq(3_000_000u32.Hz()) };

                self.cs.set_low().map_err(|e| Error::Pin(e))?;

                macro_rules! xchg {
                    ($byte:expr) => {
                        match self
                            .spi
                            .transfer(&mut [$byte, 0, 0])
                            .map_err(|e| Error::Transfer(e))?
                        {
                            [_, h, l] => ((*h as u32) << 8) | (*l as u32),
                            _ => return Err(Error::InternalError),
                        }
                    };
                }

                let z1 = xchg!(CMD_Z1_READ);
                let z2 = xchg!(CMD_Z2_READ);
                let z = z1 as i32 - z2 as i32;

                if z < threshold {
                    xchg!(0);
                    self.cs.set_high().map_err(|e| Error::Pin(e))?;
                    unsafe { set_spi_freq(62_500_000.Hz()) };
                    return Ok(None);
                }

                xchg!(CMD_X_READ | 1); // Dummy read, first read is a outlier

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

                xchg!(0);
                self.cs.set_high().map_err(|e| Error::Pin(e))?;
                unsafe { set_spi_freq(62_500_000.Hz()) };

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
        InternalError,
    }

    unsafe fn set_spi_freq(freq: impl Into<fugit::Hertz<u32>>) {
        use rp_pico::hal;
        // FIXME: the touchscreen and the LCD have different frequencies, but we cannot really set different frequencies to different SpiProxy without this hack
        hal::spi::Spi::<_, _, 8>::new(hal::pac::Peripherals::steal().SPI1)
            .set_baudrate(125_000_000u32.Hz(), freq);
    }
}
