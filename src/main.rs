#![no_std]
#![no_main]

use core::fmt::Debug;

use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    watch::Watch,
};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output},
    ledc::{
        channel::{self, ChannelIFace, Channel},
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    peripherals::Peripherals,
    time::RateExtU32,
    timer::timg::TimerGroup,
    usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx},
    Async,
};
use heapless::{String, Vec};
use libm::sqrtf;
use static_cell::StaticCell;

const MAX_BUFFER_SIZE: usize = 512;

type MutexMotor = &'static Mutex<NoopRawMutex, Motor<'static>>;

struct Motor<'d> {
    step_pin: Output<'d>,
    direction: Output<'d>,
}

#[embassy_executor::task(pool_size = 3)]
async fn move_motor_task(motor: MutexMotor, steps: i32) {
    move_motor(motor, steps).await;
}
//#[embassy_executor::task(pool_size = 3)]
async fn move_motor(
    motor: MutexMotor,
    steps: i32, /*, mut cancel: AnonReceiver<'static, CriticalSectionRawMutex, bool, 2>*/
) {
    if let Ok(mut motor) = motor.try_lock() {
        motor.direction.set_level(Level::from(steps > 0));

        let steps: u64 = steps.abs().try_into().unwrap();
        for i in 0..steps {
            let mut delay: u64 = 200;
            if i < 1000 {
                delay = 1200 - i;
            } else if steps - i < 1000 {
                delay = 1200 - (steps - i);
            } /*
              if let Some(true) = cancel.try_changed() {
                  return;
              }*/
            motor.step_pin.toggle();
            Timer::after_micros(delay).await;
        }
    } else {
        esp_println::println!("[Motor is already in use!]");
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn move_xy_task(
    motor_x: MutexMotor,
    motor_y: MutexMotor,
    steps_x: i32,
    steps_y: i32,
    time_micros: u64,
) {
    move_xy(motor_x, motor_y, steps_x, steps_y, time_micros).await;
}

async fn move_xy(
    motor_x: MutexMotor,
    motor_y: MutexMotor,
    steps_x: i32,
    steps_y: i32,
    time_micros: u64,
) {
    if let (Ok(mut motor_x), Ok(mut motor_y)) = (motor_x.try_lock(), motor_y.try_lock()) {
        motor_x.direction.set_level(Level::from(steps_x > 0));
        motor_y.direction.set_level(Level::from(steps_y > 0));
        let steps_x: u64 = steps_x.abs().try_into().unwrap();
        let steps_y: u64 = steps_y.abs().try_into().unwrap();

        let begin = Instant::now().as_micros();

        let mut x: u64 = 0;
        let mut y: u64 = 0;

        loop {
            let now = Instant::now().as_micros();
            //TODO: use something more smooth than lerp, possibly smoothstep
            let expected_x: u64 = (now - begin) * steps_x / time_micros;
            let expected_y: u64 = (now - begin) * steps_y / time_micros;
            if expected_x > x {
                motor_x.step_pin.set_high();
                x += 1;
            }
            if expected_y > y {
                motor_y.step_pin.set_high();
                y += 1;
            }
            Timer::after_micros(2).await;
            motor_x.step_pin.set_low();
            motor_y.step_pin.set_low();

            if expected_x <= x && expected_y <= y {
                break;
            }

            Timer::after_micros(40).await;
        }
    } else {
        esp_println::println!("[Motor is already in use!]");
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn square(
    motor_x: MutexMotor,
    motor_y: &'static Mutex<NoopRawMutex, Motor<'static>>,
    steps: i32,
) {
    move_motor(motor_x, steps).await;
    move_motor(motor_y, steps).await;
    move_motor(motor_x, -steps).await;
    move_motor(motor_y, -steps).await;
}

enum GCode {
    G0 { x: f32, y: f32, f: Option<f32> },
    G1 { x: f32, y: f32, f: Option<f32> },
    G90,
    G91,
    M3,
    M5,
}

enum GCodeParsingError {
    ReadError,
    ParsingError,
    MissingArgumentError,
    VectorFullError,
    InvalidFloatError,
    MutexLock,
}

impl Debug for GCodeParsingError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            GCodeParsingError::ReadError => f.write_str("Read error"),
            GCodeParsingError::ParsingError => f.write_str("Parsing error"),
            GCodeParsingError::MissingArgumentError => f.write_str("Missing argument error"),
            GCodeParsingError::VectorFullError => f.write_str("GCode vector is full"),
            GCodeParsingError::InvalidFloatError => f.write_str("Invalid float argument"),
            GCodeParsingError::MutexLock => f.write_str("Can't lock the instructions mutex")
        }
    }
}

async fn parse_gcode<'a>(
    rx: &mut UsbSerialJtagRx<'a, Async>,
    instructions: &'static Mutex<NoopRawMutex, Vec<GCode, 10000>>
) -> Result<(), GCodeParsingError> {
    //let mut instructions: Vec<GCode, 10000> = Vec::new();
    if let Ok(mut instructions) = instructions.try_lock() {
        let mut rbuf = [0u8; MAX_BUFFER_SIZE];
        let mut command: String<64> = String::new();
        loop {
            let len = embedded_io_async::Read::read(rx, &mut rbuf)
                .await
                .map_err(|_| GCodeParsingError::ReadError)?;
            let mut string_buffer: Vec<_, MAX_BUFFER_SIZE> = Vec::new();
            string_buffer.extend_from_slice(&rbuf[..len]).unwrap();
            let message = String::from_utf8(string_buffer).unwrap();
            if message == "\r" {
                let mut iter = command.split(' ');
                match iter.next() {
                    Some("G0") => {
                        let mut x: Option<f32> = None;
                        let mut y: Option<f32> = None;
                        let mut f: Option<f32> = None;
                        while let Some(arg) = iter.next() {
                            if arg.starts_with("X") {
                                x = Some(arg[1..].parse().map_err(|_| GCodeParsingError::InvalidFloatError)?);
                            } else if arg.starts_with("Y") {
                                y = Some(arg[1..].parse().map_err(|_| GCodeParsingError::InvalidFloatError)?);
                            } else if arg.starts_with("F") {
                                f = Some(arg[1..].parse().map_err(|_| GCodeParsingError::InvalidFloatError)?);
                            }
                        }

                        instructions.push(GCode::G0 {
                            x: x.ok_or(GCodeParsingError::MissingArgumentError)?,
                            y: y.ok_or(GCodeParsingError::MissingArgumentError)?,
                            f,
                        }).map_err(|_| GCodeParsingError::VectorFullError)?;
                    },
                    Some("G1") => {
                        let mut x: Option<f32> = None;
                        let mut y: Option<f32> = None;
                        let mut f: Option<f32> = None;
                        while let Some(arg) = iter.next() {
                            if arg.starts_with("X") {
                                x = Some(arg[1..].parse().map_err(|_| GCodeParsingError::InvalidFloatError)?);
                            } else if arg.starts_with("Y") {
                                y = Some(arg[1..].parse().map_err(|_| GCodeParsingError::InvalidFloatError)?);
                            } else if arg.starts_with("F") {
                                f = Some(arg[1..].parse().map_err(|_| GCodeParsingError::InvalidFloatError)?);
                            }
                        }

                        instructions.push(GCode::G1 {
                            x: x.ok_or(GCodeParsingError::MissingArgumentError)?,
                            y: y.ok_or(GCodeParsingError::MissingArgumentError)?,
                            f,
                        }).map_err(|_| GCodeParsingError::VectorFullError)?;
                    },
                    Some("G90") => instructions.push(GCode::G90).map_err(|_| GCodeParsingError::VectorFullError)?,
                    Some("G91") => instructions.push(GCode::G91).map_err(|_| GCodeParsingError::VectorFullError)?,
                    Some("M3") => instructions.push(GCode::M3).map_err(|_| GCodeParsingError::VectorFullError)?,
                    Some("M5") => instructions.push(GCode::M5).map_err(|_| GCodeParsingError::VectorFullError)?,
                    Some(_) => return Err(GCodeParsingError::ParsingError),
                    None => {}
                }
                command.clear();
            } else if message == "$" {
                break;
            } else {
                let _ = command.push_str(&message);
            }
        }

        return Ok(());
    } else {
        return Err(GCodeParsingError::MutexLock);
    }
}

const MM_PER_REVOLUTION: f32 = 1.25;
const STEPS_PER_REVOLUTION: f32 = 200.0 * 16.0;
const STEPS_PER_MM: f32 = STEPS_PER_REVOLUTION / MM_PER_REVOLUTION;

#[embassy_executor::task]
async fn run_gcode(instructions: &'static Mutex<NoopRawMutex, Vec<GCode, 10000>>, motor_x: MutexMotor, motor_y: MutexMotor, pwm_channel: &'static Mutex<NoopRawMutex, Channel<'static, LowSpeed>>) {
    if let (Ok(instructions), Ok(pwm_channel)) = (instructions.try_lock(), pwm_channel.try_lock()) {
        let mut x_pos: f32 = 0.0;
        let mut y_pos: f32 = 0.0;
        let mut absolute_position: bool = true;
        let mut speed_mm_per_min: f32 = 300.0;

        for instruction in instructions.iter() {
            match instruction {
                GCode::G0 { x: x_move, y: y_move, f } => {
                    let x = if absolute_position {x_move - x_pos} else {*x_move};
                    let y = if absolute_position {y_move - y_pos} else {*y_move};
                    let steps_x = (x * STEPS_PER_MM) as i32;
                    let steps_y = (y * STEPS_PER_MM) as i32;
                    if let Some(f) = f {
                        speed_mm_per_min = *f;
                    }

                    let length = sqrtf(x*x + y*y);
                    let time_micros = (length / speed_mm_per_min * 60000000.0) as u64;
                    move_xy(motor_x, motor_y, steps_x, steps_y, time_micros).await;
                    x_pos += x_move;
                    y_pos += y_move;
                },
                GCode::G1 { x: x_move, y: y_move, f } => {
                    let x = if absolute_position {x_move - x_pos} else {*x_move};
                    let y = if absolute_position {y_move - y_pos} else {*y_move};
                    let steps_x = (x * STEPS_PER_MM) as i32;
                    let steps_y = (y * STEPS_PER_MM) as i32;
                    if let Some(f) = f {
                        speed_mm_per_min = *f;
                    }

                    let length = sqrtf(x*x + y*y);
                    let time_micros = (length / speed_mm_per_min * 60000000.0) as u64;
                    move_xy(motor_x, motor_y, steps_x, steps_y, time_micros).await;
                    x_pos += x_move;
                    y_pos += y_move;
                },
                GCode::G90 => absolute_position = true,
                GCode::G91 => absolute_position = false,
                GCode::M3 => pwm_channel.set_duty(5).unwrap(),
                GCode::M5 => pwm_channel.set_duty(10).unwrap()
            }
        }
    } else {
        esp_println::println!("Can't lock the mutex");
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::println!("[Init!]");
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

    let (mut rx, _tx) = UsbSerialJtag::new(peripherals.USB_DEVICE)
        .into_async()
        .split();

    static CANCEL_WATCH: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();
    let cancel_sender = CANCEL_WATCH.sender();

    static INSTRUCTIONS: StaticCell<Mutex<NoopRawMutex, Vec<GCode, 10000>>> = StaticCell::new();
    let instructions = INSTRUCTIONS.init(Mutex::new(Vec::new()));

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    static LSTIMER: StaticCell<esp_hal::ledc::timer::Timer<'static, LowSpeed>> = StaticCell::new();
    let lstimer = LSTIMER.init(ledc.timer::<LowSpeed>(timer::Number::Timer0));
    lstimer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50.Hz(),
        })
        .unwrap();

    static PWM_CHANNEL: StaticCell<Mutex<NoopRawMutex, Channel<'static, LowSpeed>>> = StaticCell::new();
    let channel = PWM_CHANNEL.init(Mutex::new(ledc.channel(channel::Number::Channel0, peripherals.GPIO4)));
    channel.lock().await.configure(channel::config::Config {
            timer: lstimer,
            duty_pct: 5,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

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
                        }
                        Some("moveX") => {
                            let steps: i32 = iter.next().unwrap().parse().unwrap();
                            spawner
                                .spawn(move_motor_task(
                                    motor_x, steps, /*, CANCEL_WATCH.anon_receiver()*/
                                ))
                                .unwrap();
                        }
                        Some("moveY") => {
                            let steps: i32 = iter.next().unwrap().parse().unwrap();
                            spawner.spawn(move_motor_task(motor_y, steps)).unwrap();
                        }
                        Some("test") => {
                            spawner.spawn(square(motor_x, motor_y, 20000)).unwrap();
                        }
                        Some("cancel") => {
                            cancel_sender.send(true);
                        }
                        Some("servo") => {
                            if let Ok(channel) = channel.try_lock() {
                                let angle: u8 = iter.next().unwrap().parse().unwrap();
                                if angle > 90 {
                                    esp_println::print!("[Invalid angle!]");
                                } else {
                                    channel.set_duty(5 + angle / 18).unwrap();
                                }
                            } else {
                                esp_println::println!("[Can't lock pwm channel mutex]");
                            }
                        }
                        Some("move") => {
                            let steps_x: i32 = iter.next().unwrap().parse().unwrap();
                            let steps_y: i32 = iter.next().unwrap().parse().unwrap();
                            const TIME_PER_STEP_MICROS: u64 = 200;
                            let length: u64 = (steps_x * steps_x + steps_y * steps_y)
                                .isqrt()
                                .try_into()
                                .unwrap();
                            spawner
                                .spawn(move_xy_task(
                                    motor_x,
                                    motor_y,
                                    steps_x,
                                    steps_y,
                                    length * TIME_PER_STEP_MICROS,
                                ))
                                .unwrap();
                        }
                        Some("gcode") => {
                            esp_println::println!(
                                "[Paste GCode, end with $ after a newline. Supports G0, G1, G90, G91, M3, M5.]"
                            );
                            match parse_gcode(&mut rx, instructions).await {
                                Ok(()) => {
                                    esp_println::println!("[GCode has been parsed without errors.]");
                                },
                                Err(err) => esp_println::println!("[{:?}]", err)
                            }
                        },
                        Some("run_gcode") => {
                            spawner.spawn(run_gcode(instructions, motor_x, motor_y, channel)).unwrap();
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
            //#[allow(unreachable_patterns)]
            Err(e) => esp_println::println!("[RX Error: {:?}]", e),
        }
    }
}
