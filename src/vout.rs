use ch32v00x_hal as hal;
use embassy_time::Instant;
use hal::{pac::TIM1, timer::PwmChannel};
use ch32v00x_hal::prelude::*;

pub enum ChargeState {
	Nop,
	Pre,
	CC,
	CV,
	Full,
	Error,
}
const CHARGE_IDLE_TIME: u32 = 30 * 60 * 1000;//30minutes
const MAX_OUT_DUTY: u16 = 8200;//30V max
pub const LI_1CELL_VOL: u16 = 4200;//4.2v
pub struct VoutPwm {
	chg_idle: u32,
	cur_duty: u16,
	enable: bool,
	pwm: PwmChannel<TIM1, 3>,
}

impl VoutPwm {
	pub fn new (pwm: PwmChannel<TIM1, 3>, cur_duty: u16) -> Self{
		VoutPwm { chg_idle: 0, cur_duty, pwm, enable: false }
	}

	fn duty_increase(&mut self, step: u16) -> bool {
		let mut new_duty = self.cur_duty + step;
		if new_duty > MAX_OUT_DUTY {
			new_duty = MAX_OUT_DUTY;
		}

		self.cur_duty = new_duty;
		self.pwm.set_duty(new_duty);

		return true;
	}

	fn duty_decrease(&mut self, mut step: u16) -> bool {
		if self.cur_duty < step {
			step = self.cur_duty;
		}
		let new_duty = self.cur_duty - step;

		self.cur_duty = new_duty;
		self.pwm.set_duty(new_duty);

		return true;
	}

	// fn duty_set(&mut self, duty: u16) -> bool{
	// 	if duty < MAX_OUT_DUTY {
	// 		self.cur_duty = duty;
	// 		self.pwm.set_duty(duty);
	// 		return true;
	// 	}
	// 	return false;
	// }

	pub fn is_running(&mut self) -> bool {
		self.pwm.get_duty() != 0
	}

	fn vol_to_duty(&mut self, vol: u16) -> u16 {
		let duty = vol as u32 * 100 / 366; //div 3.66

		duty as u16
	}

	fn duty_to_vol(&mut self, vol: u16) -> u16 {
		let duty = vol as u32 * 366 / 100; //mul 3.66

		duty as u16
	}

	pub fn control(&mut self, target: u16, curr_vol: u16) {
		if !self.enable {
			return;
		}

		let delta_vol = target as i32 - curr_vol as i32;
		let abs_delta_vol = i32::abs(delta_vol) as u16;

		if abs_delta_vol <= 8 {//8mv = 2duty
			return;
		}

		let delta_duty = self.vol_to_duty(abs_delta_vol)/2;
		if delta_vol > 0 {
			self.duty_increase(delta_duty);
		} else {
			self.duty_decrease(delta_duty);
		}
	}

	pub fn output(&mut self, enable: bool) {
		self.enable = enable;
		if !enable {
			self.pwm.set_duty(0);
			self.cur_duty = 0;
		} else {
			self.chg_idle = Instant::now().as_millis() as u32;
		}
	}

	pub fn charger(&mut self, charge_curr: u16, current_ma: u16, output_vol: u16, cell: u16) -> ChargeState {
		const LI_1CELL_PRECHG_VOL: u16 = 3000;//3v
		const LI_1CELL_PRECHG_CUR: u16 = 100;//mA
		const LI_1CELL_STOPCHG_CUR:u16 = 50;//mA
		const LI_CHARGE_LAZY:u16 = 40;//mA

		let limit_vol = LI_1CELL_VOL * cell + 50;
		if output_vol > limit_vol {
			self.output(false);
			return ChargeState:: Error;
		}
		if !self.enable {
			return ChargeState::Nop;
		}

		let max_vol = (LI_1CELL_VOL - 15) * cell;
		if output_vol >= max_vol - 30 {
			if current_ma < LI_1CELL_STOPCHG_CUR {
				let now = Instant::now().as_millis() as u32;
				if now > (self.chg_idle + CHARGE_IDLE_TIME) {
					self.output(false);
					return ChargeState::Full;
				}
			} else {
				self.chg_idle = Instant::now().as_millis() as u32;
				if output_vol >= max_vol || current_ma > charge_curr + LI_CHARGE_LAZY{
					self.duty_decrease(5);//18mV
				}
			}
			return ChargeState::CV;
		}

		let prechg_vol = LI_1CELL_PRECHG_VOL * cell;
		let curr_duty = self.pwm.get_duty();
		if self.duty_to_vol(curr_duty) < prechg_vol {
			let mini_duty = self.vol_to_duty(output_vol);
			if curr_duty < mini_duty {
				self.duty_increase(mini_duty - curr_duty + 3);
				return ChargeState::Pre;
			}
		}

		let chg_curr;
		let chg_state;
		if output_vol < prechg_vol {
			chg_curr = LI_1CELL_PRECHG_CUR;
			chg_state = ChargeState::Pre;
		} else {
			chg_curr = charge_curr;
			chg_state = ChargeState::CC;
		}

		let step = self.vol_to_duty(50);
		if current_ma < chg_curr - LI_CHARGE_LAZY {
			self.duty_increase(step );
		} else if current_ma > chg_curr + LI_CHARGE_LAZY {
			self.duty_decrease(step/2);
		}

		return chg_state;
	}
}

