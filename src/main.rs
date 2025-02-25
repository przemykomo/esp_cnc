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
    gpio::{Level, Output}, ledc::{channel::{self, ChannelIFace}, timer::{self, TimerIFace}, LSGlobalClkSource, Ledc, LowSpeed}, peripherals::Peripherals, time::RateExtU32, timer::timg::TimerGroup, usb_serial_jtag::UsbSerialJtag
};
use heapless::{String, Vec};
use static_cell::StaticCell;

const MAX_BUFFER_SIZE: usize = 512;

struct Motor<'d> {
    step_pin: Output<'d>,
    direction: Output<'d>
}

#[embassy_executor::task(pool_size = 3)]
async fn move_motor(motor: &'static Mutex<NoopRawMutex, Motor<'static>>, steps: i32, mut cancel: AnonReceiver<'static, CriticalSectionRawMutex, bool, 2>) {
    if let Ok(mut motor) = motor.try_lock() {
        if steps > 0 {
            motor.direction.set_high();
        } else {
            motor.direction.set_low();
        }

        let steps: u64 = steps.abs().try_into().unwrap();
        for i in 0..steps {
            let mut delay: u64 = 200;
            if i < 1000 {
                delay = 1200 - i;
            } else if steps - i < 1000 {
                delay = 1200 - (steps - i);
            }
            if let Some(true) = cancel.try_changed() {
                return;
            }
            motor.step_pin.toggle();
            Timer::after(Duration::from_micros(delay)).await;
        }
    } else {
        esp_println::println!("Motor is already in use!");
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn square(motor_x: &'static Mutex<NoopRawMutex, Motor<'static>>, motor_y: &'static Mutex<NoopRawMutex, Motor<'static>>, steps: i32, mut cancel: AnonReceiver<'static, CriticalSectionRawMutex, bool, 2>) {
    move_motor(motor_x, steps, cancel).;
    move_motor(motor_y, steps, cancel).await;
    move_motor(motor_x, -steps, cancel).await;
    move_motor(motor_y, -steps, cancel).await;
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals: Peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    static MOTOR_X: StaticCell<Mutex<NoopRawMutex, Motor<'static>>> = StaticCell::new();
    static MOTOR_Y: StaticCell<Mutex<NoopRawMutex, Motor<'static>>> = StaticCell::new();
    let motor_x = MOTOR_X.init(Mutex::new(Motor {
        step_pin: Output::new(peripherals.GPIO0, Level::Low),
        direction: Output::new(peripherals.GPIO1, Level::Low),
    }));

    let motor_y = MOTOR_Y.init(Mutex::new(Motor {
        step_pin: Output::new(peripherals.GPIO2, Level::Low),
        direction: Output::new(peripherals.GPIO3, Level::Low),
    }));

    let (mut rx, _tx) = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async().split();

    static CANCEL_WATCH: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();
    let cancel_sender = CANCEL_WATCH.sender();

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    static LSTIMER: StaticCell<esp_hal::ledc::timer::Timer<'static, LowSpeed>> = StaticCell::new();
    let lstimer = LSTIMER.init(ledc.timer::<LowSpeed>(timer::Number::Timer0));
    lstimer.configure(timer::config::Config {
        duty: timer::config::Duty::Duty14Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: 50.Hz()
    }).unwrap();
    let mut channel = ledc.channel(channel::Number::Channel0, peripherals.GPIO4);
    channel.configure(channel::config::Config {
        timer: lstimer,
        duty_pct: 5,
        pin_config: channel::config::PinConfig::PushPull
    }).unwrap();

    let mut led = Output::new(peripherals.GPIO8, Level::Low);
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
                        Some("moveX") => {
                            let steps: i32 = iter.next().unwrap().parse().unwrap();
                            spawner.spawn(move_motor(motor_x, steps, CANCEL_WATCH.anon_receiver())).unwrap();
                        },
                        Some("moveY") => {
                            let steps: i32 = iter.next().unwrap().parse().unwrap();
                            spawner.spawn(move_motor(motor_y, steps, CANCEL_WATCH.anon_receiver())).unwrap();
                        },
                        Some("test") => {
                            spawner.spawn(move_motor(motor_x, 20000, CANCEL_WATCH.anon_receiver())).unwrap();
                            spawner.spawn(move_motor(motor_y, 20000, CANCEL_WATCH.anon_receiver())).unwrap();
                        },
                        Some("cancel") => {
                            cancel_sender.send(true);
                        },
                        Some("servo") => {
                            let angle: u8 = iter.next().unwrap().parse().unwrap();
                            if angle > 90 {
                                esp_println::print!("[Invalid angle!]");
                            } else {
                                channel.set_duty(5 + angle / 18).unwrap();
                            }
                        }
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
