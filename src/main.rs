#![no_std]
#![no_main]

mod lcd1621;
mod key;
mod vout;
mod embassy_tim_drv;

use lcd1621::Tip;
use key::KeyCode;
use ch32v00x_hal as hal;
use ch32v00x_hal::prelude::*;
use hal::{
    pac::{self, TIM1}, println, rcc::Enable, watchdog,
    timer::{Channel, Event, PwmChannel, PwmExt, Tim1FullRemap, TimerExt}
};

use panic_halt as _;

use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use embassy_futures::{self, select::Either};
use heapless::spsc::{Queue, Producer};
use static_cell::StaticCell;

static CMD_QUEUE: StaticCell<Queue<Command, 4>> = StaticCell::new();

#[derive(PartialEq)]
enum State {
    Off, Out, In, Set
}

#[derive(PartialEq, Clone)]
enum WorkMode {
    AdjPwrSup,
    LiCell(u8)
}

struct PowerData{
    vref_data: [u16; SAMPLE_NUM],
    vin_data : [u16; SAMPLE_NUM],
    vout_data: [u16; SAMPLE_NUM],
    curr_data: [u16; SAMPLE_NUM],
    index: usize,
    max_sample: u16,
    full: bool,
}
const LI_CHG_CELL_MAX: u8 = 7;
const SAMPLE_NUM: usize = 25;
const MAIN_TASK_PERIOD: u32 = 20;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    //hal::debug::SDIPrint::enable();
    let p = pac::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    rcc.config.enable_lsi = true;
    let clocks = rcc.config.freeze();

    pac::AFIO::enable(&mut rcc.apb2);
    let gpioa = p.GPIOA.split(&mut rcc);
    let gpioc = p.GPIOC.split(&mut rcc);
    let gpiod = p.GPIOD.split(&mut rcc);

    //PIN
    let pin_pwm = gpiod.pd4.into_push_pull_output_in_state(hal::gpio::PinState::Low);
    let mut pin_led_on = gpioc.pc7.into_push_pull_output_in_state(hal::gpio::PinState::High);
    let _pin_vin = gpiod.pd2.into_analog();
    let _pin_chan4 = gpiod.pd3.into_analog();
    let _pin_chan5 = gpiod.pd5.into_analog();
    let _pin_chan6 = gpiod.pd6.into_analog();
    
    let key_up = gpioc.pc6.into_pull_up_input();
    let key_down = gpioc.pc5.into_pull_up_input();
    let key_onoff = gpioc.pc4.into_pull_up_input();
    let key_inout = gpioc.pc3.into_pull_up_input();
    let rotary_a = gpiod.pd0.into_pull_up_input();
    let rotary_b = gpioc.pc2.into_pull_up_input();

    let rotary = key::RotaryEnc::new(rotary_a, rotary_b);
    let key = key::Key::new(key_up, key_down, key_onoff, key_inout);

    let mut adc = hal::adc::Adc::new(p.ADC1, &clocks);

    //PWM
    let pin_pwm = pin_pwm.into_alternate();
    let mut pwm = p.TIM1
                .pwm_hz::<Tim1FullRemap, _, _>(pin_pwm, 2400.Hz(), &clocks);
    pwm.set_duty(Channel::C4, 0);
    pwm.enable(Channel::C4);
    let pwm_ch :PwmChannel<TIM1, 3> = pwm.split();
    let mut vout = vout::VoutPwm::new(pwm_ch, 0);

    //LCD
    let lcd_cs = gpioc.pc0.into_push_pull_output();
    let lcd_wr = gpioa.pa2.into_push_pull_output();
    let lcd_data = gpioa.pa1.into_push_pull_output();
    
    let delay1 = hal::delay::CycleDelay::new(&clocks);
    let mut lcd = lcd1621::LCD1621::new(lcd_cs, lcd_wr, lcd_data, delay1);
    lcd.init();
    lcd.clear();
    lcd.set_current(0);
    lcd.set_voltage(0);
    lcd.flush();
    
    #[cfg(feature = "time_driver_systick")]
    {
        let mut systk = p.SYSTICK.counter_us(&clocks);
        systk.start(1.millis()).unwrap();
        systk.listen(SysEvent::Update);
        unsafe {
            qingke::pfic::set_priority(CoreInterrupt::SysTick as u8 as u8, qingke::interrupt::Priority::P15 as _);
            qingke::pfic::enable_interrupt(CoreInterrupt::SysTick as u8);
        }
    }

    #[cfg(feature = "time_driver_tim2")]
    {
        unsafe { qingke::pfic::enable_interrupt(pac::Interrupt::TIM2 as u8); }
        let mut tim2 = p.TIM2.counter_us(&clocks);
        tim2.start(1.millis()).unwrap();
        tim2.listen(Event::Update);
    }
    
    let cmd_queue = CMD_QUEUE.init(Queue::new());
    let (cmd_send, mut cmd_recv) = cmd_queue.split();

    let mut wdg = watchdog::IndependentWatchdog::new(p.IWDG);
    wdg.start(250.millis());
    
    //let _ = spawner.spawn(task_rotary(, enc_send));
    let _ = spawner.spawn(task_key(key, rotary, cmd_send));

    let mut pwr_data = PowerData {
        vref_data: [0; SAMPLE_NUM],
        vin_data : [0; SAMPLE_NUM],
        vout_data: [0; SAMPLE_NUM],
        curr_data: [0; SAMPLE_NUM],
        index: 0,
        max_sample: adc.max_sample(),
        full: false,
    };
    //init vref
    for vref in pwr_data.vref_data.as_mut() {
        *vref = adc.read_vref();
    }

    let mut output_vol;
    let mut input_vol = 0;
    let mut target_vol = 0;

    let mut mode: WorkMode = WorkMode::AdjPwrSup;
    let mut state: State = State::Off;
    let mut last_state: State = State::Off;
    let mut flash_flag = 0;

    const PAUSE_TIME: u32 = 1500;
    let mut pause_time = 0;

    let mut charge_curr: u16 = 500;
    let mut charge_ma_ms: u64 = 0;
    let mut charge_time: Instant = Instant::now();
    let mut command;
    loop {
        wdg.feed();

        command = None;
        if cmd_recv.ready() {
            command = cmd_recv.dequeue();
        }
        
        if let Some(cmd) = command {
            match cmd {
                Command::VoutInc(mut step) => if state != State::In  {
                    pause_time = PAUSE_TIME;
                    if state != State::Set {
                        last_state = state;
                        step = 0;
                    }
                    if mode == WorkMode::AdjPwrSup {
                        if target_vol + step < input_vol {
                            target_vol += step;
                        }
                        lcd.set_voltage(target_vol);
                    } else {
                        if charge_curr + step <= 2000 {
                            charge_curr += step;
                        }
                        lcd.set_current(charge_curr);
                    }
                    state = State::Set;
                    lcd.set_tip(Tip::Set);
                },
                Command::VoutDec(mut step) => if state != State::In {
                    pause_time = PAUSE_TIME;
                    if state != State::Set {
                        last_state = state;
                        step = 0;
                    }
                    if mode == WorkMode::AdjPwrSup {
                        if target_vol > step {
                            target_vol -= step;
                        } else if target_vol != 0{
                            target_vol = 0;
                        }
                        lcd.set_voltage(target_vol);
                    } else {
                        if charge_curr > step {
                            charge_curr -= step;
                        }
                        if charge_curr < 100 {
                            charge_curr = 100; //100mA mini
                        }
                        lcd.set_current(charge_curr);
                    }
                    state = State::Set;
                    lcd.set_tip(Tip::Set);
                },
                Command::DispOutVol => {
                    state = State::Out;
                    if vout.is_running() {
                        lcd.set_tip(Tip::Out);
                    } else {
                        lcd.set_tip(Tip::None);
                    }
                },
                Command::DispInVol => {
                    state = State::In;
                    lcd.set_tip(Tip::In);
                },
                Command::OutputOn => {
                    vout.output(true);
                    pin_led_on.set_low();
                    state = State::Out;
                    lcd.set_tip(Tip::Out);
                    charge_ma_ms = 0;
                    charge_time = Instant::now();
                },
                Command::OutputOff => {
                    vout.output(false);
                    pin_led_on.set_high();
                    state = State::Off;
                    lcd.set_tip(Tip::None);
                },
                Command::SwitchMode(init) => {
                    let lcd_vol;
                    match mode {
                        WorkMode::AdjPwrSup => {
                            if !init {
                                mode = WorkMode::LiCell(1);
                                lcd_vol = vout::LI_1CELL_VOL;
                            } else {
                                lcd_vol = target_vol;
                            }
                        },
                        WorkMode::LiCell(mut cell) => {
                            if init {
                                cell = LI_CHG_CELL_MAX;
                            }
                            let next_vol = (cell + 1) as u16 * vout::LI_1CELL_VOL;
                            if cell < LI_CHG_CELL_MAX && next_vol < input_vol{
                                mode = WorkMode::LiCell(cell+1);
                                lcd_vol = next_vol;
                            } else {
                                mode = WorkMode::AdjPwrSup;
                                lcd_vol = target_vol;
                            }
                        },
                    }
                    pause_time = PAUSE_TIME;
                    state = State::Set;
                    lcd.set_tip(Tip::Set);
                    lcd.set_voltage(lcd_vol);
                    lcd.set_current(if lcd_vol == target_vol { 0 } else { charge_curr });
                    vout.output(false);
                    last_state = State::Off;
                    charge_ma_ms = 0;
                }
            }

            lcd.flush();
        }
        
        if pause_time == 0 {
            if pwr_data.full {
                pwr_data.full = false;
                
                input_vol = cal_average_data(&pwr_data.vin_data);
                output_vol = cal_average_data(&pwr_data.vout_data);
                if state == State::In {
                    lcd.set_voltage(input_vol)
                } else {
                    if vout.is_running() {
                        lcd.set_voltage(output_vol)
                    } else {
                        match mode {
                            WorkMode::AdjPwrSup => { lcd.set_voltage(output_vol); },
                            WorkMode::LiCell(_) => {
                                lcd.clr_voltage();
                                flash_flag = 40/MAIN_TASK_PERIOD;//100ms
                            },
                        };
                    };
                }

                let curr = cal_average_data(&pwr_data.curr_data);
                match mode {
                    WorkMode::AdjPwrSup => {
                        lcd.set_current(curr);
                        vout.control(target_vol, output_vol);
                    },
                    WorkMode::LiCell(cell) => {
                        if vout.is_running() {
                            lcd.set_current(curr);

                            let elapsed_ms = charge_time.elapsed().as_millis();
                            charge_time = Instant::now();
                            charge_ma_ms += curr as u64 * elapsed_ms;
                        } else {
                            if charge_ma_ms == 0 {
                                lcd.set_current(charge_curr);
                            } else {
                                lcd.set_capacity((charge_ma_ms/3_600_000) as u16)
                            }
                        }

                        let _ = vout.charger(charge_curr, curr, output_vol, cell as u16);
                    },
                };

                lcd.flush();
            }
        } else {
            if pause_time > MAIN_TASK_PERIOD {
                pause_time -= MAIN_TASK_PERIOD;
            } else {
                pause_time = 0;

                if last_state == State::Out {
                    state = State::Out;
                    lcd.set_tip(Tip::Out);
                } else {
                    lcd.set_tip(Tip::None);
                }
                lcd.flush();
            }
        }
        
        Timer::after_ticks(MAIN_TASK_PERIOD as u64).await;

        pwr_process(&mut adc, &mut pwr_data);

        if flash_flag != 0 {
            if flash_flag == 1 {
                if let WorkMode::LiCell(cell) = mode {
                    lcd.set_voltage(vout::LI_1CELL_VOL * cell as u16);
                    lcd.flush();
                };
            }
            flash_flag -= 1;
        }
    }
}
//--------------------------------------------------------------------
enum Command {
    VoutInc(u16),
    VoutDec(u16),
    DispOutVol,
    DispInVol,
    OutputOn,
    OutputOff,
    SwitchMode(bool)
}

#[embassy_executor::task]
async fn task_key(mut key: key::Key, mut rotary: key::RotaryEnc, mut cmd: Producer<'static, Command, 4>) {
    let mut disp_out = true;
    let mut output_on = false;
    let mut inst: Instant = Instant::now();
    let mut step: u16 = 50;
    loop {
        let key_code = match embassy_futures::select::select(Timer::after_ticks(50), rotary.wait()).await {
            Either::First(_) => key.read().await,
            Either::Second(key_r) => {
                if inst.elapsed().as_millis() < 300 {
                    step = 200;
                } else {
                    step = 50;
                }
                inst = Instant::now();

                key_r
            },
        };

        match key_code {
            KeyCode::KeyNone => {},
            KeyCode::KeyUp(lp) => {
                let _ = cmd.enqueue( if lp { Command::VoutInc(1000) } else { Command::VoutInc(100) } );
            },
            KeyCode::KeyDown(lp) =>  {
                let _ = cmd.enqueue( if lp { Command::VoutDec(1000) } else { Command::VoutDec(100) } );
            },
            KeyCode::KeyOnOff(lp) =>  {
                if lp {
                    let _ = cmd.enqueue(Command::SwitchMode(true));
                    key.wait_up().await;
                } else {
                    output_on = !output_on;
                    let _ = cmd.enqueue( if output_on { Command::OutputOn } else { Command::OutputOff } );
                }
            },
            KeyCode::KeyInOut(lp) =>  {
                if lp {
                    let _ = cmd.enqueue(Command::SwitchMode(false));
                    key.wait_up().await;
                } else {
                    disp_out = !disp_out;
                    let _ = cmd.enqueue( if disp_out { Command::DispOutVol } else { Command::DispInVol } );
                }
            },
            KeyCode::RotaryCw => {
                let _ = cmd.enqueue( Command::VoutInc(step) );
            },
            KeyCode::RotaryCcw => {
                let _ = cmd.enqueue( Command::VoutDec(step) );
            },
            KeyCode::KeyCombine(_) => {},
        }
    }
}
//--------------------------------------------------------------------
const VREF_VOL: u32 = 1206; //mV

enum AdcChan {
	Vin = 3,
	Vout = 6,
	Curr = 4
}

#[inline]
fn cal_vcc_vol(data: u16, max_sample: u16) -> u16 {
    (max_sample as u32 * VREF_VOL / data as u32) as u16
}
#[inline]
fn cal_vin_vout_vol(data: u16, max_sample: u16, vcc: u16) -> u16 {
    let val = (data as u32) * (vcc as u32)/(max_sample as u32);
    let vol = (18000 + 1800) * val / 1800;

    vol as u16
}
#[inline]
fn cal_current(data: u16, max_sample: u16, vcc: u16) -> u16 {
    let uv = 1000 * (data as u32) * (vcc as u32) / (max_sample as u32);
    let mut vcurr = 330 * uv / (8200 + 330);

    if uv < 26 {
        vcurr = 72 * uv / 26;
    } else {
        vcurr = (vcurr * 98 / 25 + 7259) / 100;
    }

    vcurr as u16 // uV/mΩ
}

fn cal_average_data(data: &[u16]) -> u16 {
    let mut sum: u32 = 0;
    for d in data {
        sum += *d as u32;
    }

    (sum / data.len() as u32) as u16
}

fn pwr_process(adc: &mut hal::adc::Adc<pac::ADC1>, pwr_data: &mut PowerData) {

    let mut index = pwr_data.index;
    let max_sample = pwr_data.max_sample;

    pwr_data.vref_data[index] = adc.read_vref();
    let vcc = cal_vcc_vol(cal_average_data(&pwr_data.vref_data), pwr_data.max_sample);
    pwr_data.vin_data[index]  = cal_vin_vout_vol(adc.convert(AdcChan::Vin as u8), max_sample, vcc);
    let mut vout = cal_vin_vout_vol(adc.convert(AdcChan::Vout as u8), max_sample, vcc);
    pwr_data.vout_data[index] = if vout >= 50 { vout += 142; vout } else { vout };//correct 140mV
    pwr_data.curr_data[index] = cal_current(adc.convert(AdcChan::Curr as u8), max_sample, vcc);

    index += 1;
    if index >= SAMPLE_NUM {
        index = 0;
        pwr_data.full = true;
    }
    pwr_data.index = index;
}
