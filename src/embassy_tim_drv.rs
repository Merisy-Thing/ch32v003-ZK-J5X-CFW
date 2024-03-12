use ch32v00x_hal as hal;
use critical_section::Mutex;
//use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use embassy_time_driver::{AlarmHandle, Driver};
use hal::{pac::TIM2};
use core::cell::Cell;
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

static SYS_TICK: AtomicU32 = AtomicU32::new(0);

#[allow(non_snake_case)]
#[cfg(feature = "time_driver_systick")]
#[no_mangle]
extern "C" fn SysTick() {
    let sys_tick = SYS_TICK.load(Ordering::Relaxed);
    SYS_TICK.store(sys_tick.wrapping_add(1), Ordering::Relaxed);
    DRIVER.check_alarm(sys_tick);
    unsafe {(*SYSTICK::PTR).sr.write(|w| w.bits(0))};
}

#[allow(non_snake_case)]
#[cfg(feature = "time_driver_tim2")]
#[no_mangle]
extern "C" fn TIM2() {
    let sys_tick = SYS_TICK.load(Ordering::Relaxed);
    SYS_TICK.store(sys_tick.wrapping_add(1), Ordering::Relaxed);
    DRIVER.check_alarm(sys_tick);
    unsafe { (*TIM2::PTR).intfr.modify(|_, w| w.uif().clear_bit()) };
}

struct AlarmState {
    timestamp: Cell<u32>,
    callback: Cell<Option<(fn(*mut ()), *mut ())>>,
}
unsafe impl Send for AlarmState {}

const DUMMY_ALARM: AlarmState = AlarmState {
    timestamp: Cell::new(0),
    callback: Cell::new(None),
};

struct TimerDriver {
    alarms: Mutex<AlarmState>,
    allocated: AtomicBool,
}

embassy_time_driver::time_driver_impl!(static DRIVER: TimerDriver = TimerDriver{
    alarms:  Mutex::new(DUMMY_ALARM),
    allocated: AtomicBool::new(false),
});

impl TimerDriver {
    fn check_alarm(&self, curr_tick: u32) {
        critical_section::with(|cs| {
            let allocated = self.allocated.load(Ordering::Relaxed);
            if !allocated {
                return;
            }
            let alarm = &self.alarms.borrow(cs);

            let timestamp = alarm.timestamp.get();

            if timestamp <= curr_tick as u32 {
                alarm.timestamp.set(u32::MAX);
        
                if let Some((f, ctx)) = alarm.callback.get() {
                    f(ctx);
                }
            }
        });
    }
}

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        SYS_TICK.load(Ordering::Relaxed) as u64
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        let allocated = self.allocated.load(Ordering::Relaxed);
        if allocated {
            return None;
        }

        self.allocated.store(true, Ordering::Relaxed);
        Some(AlarmHandle::new(0))
    }

    fn set_alarm_callback(&self, _alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs);
            alarm.callback.set(Some((callback, ctx)));
        })
    }

    fn set_alarm(&self, _alarm: AlarmHandle, timestamp: u64) -> bool {
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs);

            let now = self.now();
            if timestamp <= now {
                alarm.timestamp.set(u32::MAX);
                false
            } else {
                alarm.timestamp.set(timestamp as u32);
                true
            }
        })
    }
}
