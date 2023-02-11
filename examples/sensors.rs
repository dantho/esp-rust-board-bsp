#![no_std]
#![no_main]

use esp_rust_board::{
    esp32c3_hal::{
        prelude::*,
        peripherals::Peripherals,
        clock::ClockControl,
        pulse_control::ClockSource,
        i2c::I2C,
        timer::TimerGroup,
        Delay, Rtc, PulseControl, IO,
        utils::{smartLedAdapter, SmartLedsAdapter},
    },
    esp_backtrace as _,
    icm42670::{
        Address,
        Icm42670,
        accelerometer::{
            Accelerometer,
            vector::{
                VectorExt,
                F32x3,
            },
            orientation::{self, Orientation},
        },
    },
    print, println,
    shared_bus::BusManagerSimple,
    shtcx::{shtc3, LowPower, PowerMode},
    smart_leds::{
        RGB8,
        brightness, gamma,
        SmartLedsWrite,
        colors,
    },
};
use owo_colors::OwoColorize;
use micromath::{F32Ext};

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    // Configure RMT peripheral globally
    let pulse = PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control,
        ClockSource::APB,
        0,
        0,
        0,
    )
    .unwrap();

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = <smartLedAdapter!(1)>::new(pulse.channel0, io.pins.gpio2);

    // Initialize the Delay peripheral and use it to toggle the LED in a loop.
    let mut delay = Delay::new(&clocks);

    // Initialize the I2C bus using GPIO10 for SDA and GPIO8 for SCL, running at
    // 400kHz.
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio10,
        io.pins.gpio8,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    // Create a bus manager so that we can share the I2C bus between sensor drivers
    // while avoiding ownership issues.
    let bus = BusManagerSimple::new(i2c);
    let mut icm = Icm42670::new(bus.acquire_i2c(), Address::Primary).unwrap();
    let mut sht = shtc3(bus.acquire_i2c());

    // The SHTC3 temperature/humidity sensor must be woken up prior to reading.
    sht.wakeup(&mut delay).unwrap();

    // // Setup Accel orientation tracker
    // let tracker = Accelerometer::Tracker();

    loop {
        // Read and display normalized accelerometer and gyroscope values.
        let accel_norm = icm.accel_norm().unwrap();
        let gyro_norm = icm.gyro_norm().unwrap();
        let dir_color = directional_color(accel_norm);

        led.write(brightness(gamma(core::iter::once(dir_color)), 20)).unwrap();

        print!(
            "ACCEL = X: {:+.04} Y: {:+.04} Z: {:+.04}   ",
            accel_norm.x, accel_norm.y, accel_norm.z
        );

        if accel_norm.magnitude() > 1.2 {
            print!(
                "Mag: {:+.04}   ",
                accel_norm.magnitude()
            );
        }

        print!(
            "GYRO = X: {:+.04} Y: {:+.04} Z: {:+.04}   ",
            gyro_norm.x, gyro_norm.y, gyro_norm.z
        );

        // Read and display temperature and relative humidity values.
        let measurement = sht.measure(PowerMode::NormalMode, &mut delay).unwrap();
        let temp_c = measurement.temperature.as_degrees_celsius();
        let temp_f = c2f(temp_c);
        if temp_f > 80.0 {
            print!("TEMP = {:+.2} °C {:+.2} °F   ", temp_c.red(), temp_f.red());
        } else {
            print!("TEMP = {:+.2} °C {:+.2} °F   ", temp_c, temp_f);
        };
        println!("RH = {:+.2} %RH", measurement.humidity.as_percent());

        // // Foreground colors
        // println!("My number is {:#x}!", 10.green());
        // // Background colors
        // println!("My number is not {}!", 4.on_red());

        delay.delay_ms(0u32);
    }
}

fn c2f(temp_c: f32) -> f32 {
    temp_c/5.0*9.0+32.0
}

fn directional_color(accel: F32x3) -> RGB8 {
    let mag = accel.magnitude();
    let F32x3 {x, y, z} = accel;
    RGB8::new(
        (x.abs()/mag * u8::MAX as f32) as u8,
        (y.abs()/mag * u8::MAX as f32) as u8,
        (z.abs()/mag * u8::MAX as f32) as u8,
    )
}

// fn directional_color(dir: Orientation) -> RGB8 {
//     match dir() {
//         Orientation::Unknown => colors::BLACK,
//         Orientation::FaceDown => colors::SANDY_BROWN,
//         Orientation::FaceUp => colors::AZURE,
//         Orientation::PortraitDown => colors::YELLOW,
//         Orientation::PortraitUp => colors::BEIGE,
//         Orientation::LandscapeDown => colors::SEA_GREEN,
//         Orientation::LandscapeUp => colors::DARK_GREEN,
//     }
// }
