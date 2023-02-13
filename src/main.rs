//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

pub mod moisture;
pub mod sum_tree;

use bsp::{
    entry,
    hal::{
        pwm::{self, CountRisingEdge},
        Adc,
    },
};
use cortex_m::{delay::Delay, prelude::_embedded_hal_adc_OneShot};
use defmt_rtt as _;
use ds18b20::{Ds18b20, Resolution};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::PwmPin;
use moisture::MoistureSensor;
use one_wire_bus::{OneWire, OneWireResult};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
use rp_pico::hal;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{clocks::Clock, pac};

use core::{
    fmt::Write,
    ops::{Add, Sub},
    time::Duration,
};
use heapless::String;

use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

fn get_temperature<P, E>(
    delay: &mut Delay,
    tx: &mut String<64>,
    one_wire_bus: &mut OneWire<P>,
) -> OneWireResult<(), E>
where
    P: OutputPin<Error = E> + InputPin<Error = E>,
    E: core::fmt::Debug,
{
    ds18b20::start_simultaneous_temp_measurement(one_wire_bus, delay)?;
    Resolution::Bits12.delay_for_measurement_time(delay);

    let mut search_state = None;
    loop {
        if let Some((device_address, state)) =
            one_wire_bus.device_search(search_state.as_ref(), false, delay)?
        {
            search_state = Some(state);
            if device_address.family_code() != ds18b20::FAMILY_CODE {
                // skip other devices
                continue;
            }
            let sensor = Ds18b20::new(device_address)?;
            let sensor_data = sensor.read_data(one_wire_bus, delay)?;
            let _ = writeln!(tx, "{}°C", sensor_data.temperature);
        } else {
            //let _ = writeln!(tx, "No data");
            //let _ = writeln!(tx, "Skip");
            break;
        }
    }
    Ok(())
}

fn pump_delay(moist_perc: u16) -> Option<u32> {
    const TARGET: u16 = 70;
    if moist_perc >= TARGET {
        return None;
    }

    Some((((TARGET - moist_perc) / 5) * 3_000) as u32)
}

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    //#[cfg(feature = "rp2040-e5")]
    //{
    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    // One wire setup
    let mut moisture_adc_pin = pins.gpio28.into_floating_input();
    //let mut one_wire_bus = OneWire::new(pins.gpio15.into_mode()).unwrap();
    let mut pump_pin = pins.gpio16.into_push_pull_output();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut last_update = timer.get_counter();
    let mut turn_off_pump = None;
    let mut moist_sensor = MoistureSensor::default();

    loop {
        if timer.get_counter().sub(last_update).to_secs() >= 5 {
            let _ = serial.write(b"Update 1-wire:\r\n");

            let mut text: String<64> = String::new();

            let v: u16 = adc.read(&mut moisture_adc_pin).unwrap();
            moist_sensor.add_sample(v);

            let _ = writeln!(
                text,
                "v: {v}, avg: {}, perc: {}%",
                moist_sensor.avg(),
                moist_sensor.avg_percent()
            );

            let _ = serial.write(text.as_bytes());

            if turn_off_pump.is_none() {
                if let Some(d) = pump_delay(moist_sensor.avg_percent()) {
                    let _ = writeln!(text, "Pumping for {d}ms");
                    let _ = serial.write(text.as_bytes());

                    let d = fugit::Duration::<u64, 1, 1_000>::millis(d as u64);
                    turn_off_pump = Some(timer.get_counter().checked_add_duration(d).unwrap());

                    let _ = pump_pin.set_high();
                }
            }

            last_update = timer.get_counter();
        }

        if let Some(t) = turn_off_pump {
            if t < timer.get_counter() {
                // Pull pin down
                let _ = pump_pin.set_low();
                let _ = serial.write(b"Finished pumping\r\n");
                turn_off_pump = None;
            }
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
}

// End of file
