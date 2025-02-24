//! This shows how to read and write text via USB Serial/JTAG using embassy.
//! You need to connect via the Serial/JTAG interface to see any output.
//! Most dev-kits use a USB-UART-bridge - in that case you won't see any output.

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s3
//% FEATURES: embassy esp-hal/unstable

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex}, mutex::Mutex, watch::{AnonReceiver, Watch}};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    peripherals::Peripherals,
    timer::timg::TimerGroup,
    usb_serial_jtag::UsbSerialJtag,
    gpio::{Level, Output},
};
use heapless::{String, Vec};
use static_cell::StaticCell;

const MAX_BUFFER_SIZE: usize = 512;

#[embassy_executor::task]
async fn move_motor(motor: &'static Mutex<NoopRawMutex, Output<'static>>, steps: i32, mut cancel: AnonReceiver<'static, CriticalSectionRawMutex, bool, 2>) {
    if let Ok(mut motor) = motor.try_lock() {
        for _ in 0..steps {
            if let Some(true) = cancel.try_changed() {
                return;
            }
            motor.toggle();
            Timer::after(Duration::from_millis(1)).await;
        }
    } else {
        esp_println::println!("Motor is already in use!");
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals: Peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let mut led = Output::new(peripherals.GPIO8, Level::Low);
    static MOTOR: StaticCell<Mutex<NoopRawMutex, Output<'static>>> = StaticCell::new();
    let motor = MOTOR.init(Mutex::new(Output::new(peripherals.GPIO0, Level::Low)));

    let (mut rx, _tx) = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async().split();

    static CANCEL_WATCH: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();
    let cancel_sender = CANCEL_WATCH.sender();
    //static SIGNAL: StaticCell<Signal<NoopRawMutex, String<MAX_BUFFER_SIZE>>> = StaticCell::new();
    //let signal = &*SIGNAL.init(Signal::new());

    //spawner.spawn(reader(rx, &signal, led)).unwrap();
    //spawner.spawn(writer(tx, &signal)).unwrap();


    let mut rbuf = [0u8; MAX_BUFFER_SIZE];
    let mut command: String<64> = String::new();

    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf).await;
        match r {
            Ok(len) => {
                let mut string_buffer: Vec<_, MAX_BUFFER_SIZE> = Vec::new();
                string_buffer.extend_from_slice(&rbuf[..len]).unwrap();
                let message = String::from_utf8(string_buffer).unwrap();
                //transmit.signal(message.clone());
                if message == "\r" {
                    esp_println::println!();
                    let mut iter = command.split(' ');
                    match iter.next() {
                        Some("led") => led.toggle(),
                        Some("blink") => {
                            let num: i32 = iter.next().unwrap().parse().unwrap();
                            for _ in 0..num {
                                led.toggle();
                                Timer::after(Duration::from_millis(500)).await;
                            }
                        },
                        Some("move") => {
                            let steps: i32 = iter.next().unwrap().parse().unwrap();
                            spawner.spawn(move_motor(motor, steps, CANCEL_WATCH.anon_receiver())).unwrap();
                        },
                        Some("cancel") => {
                            cancel_sender.send(true);
                        },
                        Some(_) => esp_println::println!("[Invalid command!]"),
                        None => {}
                    }
                    command.clear();
                } else if message.as_bytes() == [8] {
                    let len = command.len();
                    command.pop();
                    esp_println::print!("\r{:len$}\r{command}", "");
                } else {
                    let _ = command.push_str(&message);
                    esp_println::print!("{}", message);
                }
            }
            #[allow(unreachable_patterns)]
            Err(e) => esp_println::println!("RX Error: {:?}", e),
        }
    }
}
