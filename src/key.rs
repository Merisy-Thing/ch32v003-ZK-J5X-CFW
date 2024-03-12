use ch32v00x_hal::gpio::Input;
use ch32v00x_hal::gpio::Pin;
use ch32v00x_hal::gpio::PullUp;
use ch32v00x_hal::pac;
use ch32v00x_hal::println;
use embassy_time::Timer;
use core::future::Future;
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

#[derive(PartialEq)]
pub enum KeyCode {
	KeyNone,
	KeyUp(bool),
	KeyDown(bool),
	KeyOnOff(bool),
	KeyInOut(bool),
	KeyCombine(u8),
    RotaryCw,
    RotaryCcw
}

static ROTARY_WAKER: AtomicWaker = AtomicWaker::new();

#[allow(non_snake_case)]
#[no_mangle]
extern "C" fn EXTI7_0() {
    let exti = unsafe { &*pac::EXTI::PTR };

    if exti.intfr.read().pr0().bit_is_set() {//PD0
        exti.intenr.modify(|_, w| w.mr0().clear_bit());//disable int
        exti.intfr.write(|w| w.pr0().set_bit());//clear int flag
        ROTARY_WAKER.wake();
    }
}

pub struct RotaryEnc {
    ra_p: Pin<'D', 0, Input<PullUp>>,
    rb_p: Pin<'C', 2, Input<PullUp>>,
}

impl RotaryEnc {
    pub fn new( ra_p: Pin<'D', 0, Input<PullUp>>, rb_p: Pin<'C', 2, Input<PullUp>> ) -> Self {
        let afio = unsafe { &*pac::AFIO::PTR };
        let exti = unsafe { &*pac::EXTI::PTR };
    
        unsafe { 
            //EXTILineConfig
            afio.exticr.modify(|_, w| w.exti0().bits(3));//PD0
            exti.ftenr.modify(|_, w| w.tr0().set_bit());
            exti.rtenr.modify(|_, w| w.tr0().set_bit());

            qingke::pfic::enable_interrupt(pac::Interrupt::EXTI7_0 as u8);
        };

        RotaryEnc { ra_p, rb_p }
    }

    #[inline]
    pub async fn wait(&mut self) -> KeyCode{
        RotaryFuture::new().await;
        Timer::after_millis(2).await;

        if self.ra_p.is_high() {
            if self.rb_p.is_high() {
                KeyCode::RotaryCcw
            } else {
                KeyCode::RotaryCw
            }
        } else {
            if self.rb_p.is_high() {
                KeyCode::RotaryCw
            } else {
                KeyCode::RotaryCcw
            }
        }
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct RotaryFuture {}

impl RotaryFuture {
    pub fn new() -> Self {
        let exti = unsafe { &*pac::EXTI::PTR };
        exti.intenr.modify(|_, w| w.mr0().set_bit());

        Self {}
    }
}

impl Future for RotaryFuture {
    type Output = ();

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        ROTARY_WAKER.register(cx.waker());
        let exti = unsafe { &*pac::EXTI::PTR };

        if exti.intenr.read().mr0().bit_is_set() {
            // not triggered yet
            Poll::Pending
        } else {
            // triggered
            Poll::Ready(())
        }
    }
}

const KEY_LONG_PRESS: u32 = 500; //ms

const KEY_NONE: u8 = 0;
const KEY_UP: u8 = 1;
const KEY_DOWN: u8 = 2;
const KEY_ONOFF: u8 = 4;
const KEY_INOUT: u8 = 8;

pub struct Key {
    up_p: Pin<'C', 6, Input<PullUp>>,
    down_p: Pin<'C', 5, Input<PullUp>>,
    onoff_p: Pin<'C', 4, Input<PullUp>>,
    inout_p: Pin<'C', 3, Input<PullUp>>,
}

impl Key {
    pub fn new(
        up_p: Pin<'C', 6, Input<PullUp>>,
        down_p: Pin<'C', 5, Input<PullUp>>,
        onoff_p: Pin<'C', 4, Input<PullUp>>,
        inout_p: Pin<'C', 3, Input<PullUp>>,
    ) -> Self {
        Key {
            up_p,
            down_p,
            onoff_p,
            inout_p,
        }
    }

    fn get_key(&mut self) -> u8 {
        let mut col = 0;

        if self.up_p.is_low() {
            col |= 1;
        }
        if self.down_p.is_low() {
            col |= 2;
        }
        if self.onoff_p.is_low() {
            col |= 4;
        }
        if self.inout_p.is_low() {
            col |= 8;
        }

        return col;
    }

    pub async fn wait_up(&mut self) {
        let mut cnt = 0;
        loop {
            if self.get_key() == KEY_NONE {
                cnt += 1;
                if cnt >= 10 {
                    break;
                }
            } else {
                cnt = 0;
            }

            Timer::after_millis(10).await;
        }
    }

    pub async fn read(&mut self) -> KeyCode {
        if self.get_key() == KEY_NONE {
            return KeyCode::KeyNone;
        }
        Timer::after_millis(40).await;
        let key = self.get_key();
        if key == KEY_NONE {
            return KeyCode::KeyNone;
        }

        let mut long = false;
        let mut cnt = 0;
        loop  {
            Timer::after_millis(10).await;
            if self.get_key() != KEY_NONE {
                cnt += 10;
                if cnt >= KEY_LONG_PRESS {
                    long = true;
                    break;
                }
            } else {
                break;
            }
        }

        match key {
			KEY_UP => KeyCode::KeyUp(long),
			KEY_DOWN => KeyCode::KeyDown(long),
			KEY_ONOFF => KeyCode::KeyOnOff(long),
			KEY_INOUT => KeyCode::KeyInOut(long),
			_ => KeyCode::KeyCombine(key),
        }
    }
}
