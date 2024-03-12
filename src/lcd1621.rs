#![allow(non_camel_case_types)]
#![allow(dead_code)]
use embedded_hal_1::digital::OutputPin;
use embedded_hal_1::delay::DelayNs;

const BIAS     :u8 = 0x52;             //0b1000 0101 0010  1/3duty 4com
const SYSDIS   :u8 = 0x00;             //0b1000 0000 0000  关振系统荡器和LCD偏压发生器
const SYSEN    :u8 = 0x02;             //0b1000 0000 0010 打开系统振荡器
const LCDOFF   :u8 = 0x04;             //0b1000 0000 0100  关LCD偏压
const LCDON    :u8 = 0x06;             //0b1000 0000 0110  打开LCD偏压
//const XTAL     :u8 = 0x28;             //0b1000 0010 1000 外部接时钟
const RC256    :u8 = 0x30;             //0b1000 0011 0000  内部时钟
//const TONEON   :u8 = 0x12;             //0b1000 0001 0010  打开声音输出
//const TONEOFF  :u8 = 0x10;             //0b1000 0001 0000 关闭声音输出
const WDTDIS1  :u8 = 0x0A;             //0b1000 0000 1010  禁止看门狗

pub struct LCD1621<CsPin, WrPin, DataPin, DLY: DelayNs> 
{
	cs_p: CsPin,
	wr_p: WrPin,
	data_p: DataPin,
	delay: DLY,
	buffer: [u8; 16],
}

impl<CsPin, WrPin, DataPin, DLY: DelayNs> LCD1621 <CsPin, WrPin, DataPin, DLY> 
where
	CsPin: OutputPin,
	WrPin: OutputPin,
	DataPin: OutputPin
{
    pub fn new( cs_p: CsPin, wr_p: WrPin, data_p: DataPin, delay: DLY) -> Self {
		LCD1621 { cs_p, wr_p, data_p, delay, buffer: [0x00; 16] }
	}

	pub fn init(&mut self) {
		let _ = self.cs_p.set_high();
		let _ = self.wr_p.set_high();
		let _ = self.data_p.set_high();

		self.wr_cmd(BIAS);
		self.wr_cmd(RC256);
		self.wr_cmd(SYSDIS);
		self.wr_cmd(WDTDIS1);
		self.wr_cmd(SYSEN);
		self.wr_cmd(LCDON);
	}

	fn wr_cmd(&mut self, cmd: u8) {  //100
		let _ = self.cs_p.set_low();
		self.wr_data(0x80, 4);
		self.wr_data(cmd, 8);
		let _ = self.cs_p.set_high();
	}

	fn wr_data(&mut self, mut data: u8, cnt: u8) {
		for _i in 0..cnt {
			let _ = self.wr_p.set_low();
			self.delay.delay_us(5);
			if (data & 0x80) > 0 {
				let _ = self.data_p.set_high();
			}
			else
			{
				let _ = self.data_p.set_low();
			}
			let _ = self.wr_p.set_high();
			self.delay.delay_us(5);
			data <<= 1;
		}
	}

	pub fn display(&mut self, on: bool) {
		self.wr_cmd(if on { LCDON } else { LCDOFF });
	}

	pub fn clear (&mut self) {
		for item in self.buffer.as_mut() {
			*item = 0;
		}
	}

	pub fn flush(&mut self) {
		let _ = self.cs_p.set_low();
		self.wr_data(0xa0, 3);
		self.wr_data(0, 6);
		for item in self.buffer {
			self.wr_data(item, 8);
		}
		let _ = self.cs_p.set_high();
	}

	fn bit_set(&mut self, idx: usize) {
		let byte_idx = idx >> 3;
		let bit_idx = idx & 0x07;
		if byte_idx < self.buffer.len() {
			self.buffer[byte_idx] |= 1 << bit_idx;
		}
	}
	fn bit_clr(&mut self, idx: usize) {
		let byte_idx = idx >> 3;
		let bit_idx = idx & 0x07;
		if byte_idx < self.buffer.len() {
			self.buffer[byte_idx] &= !(1 << bit_idx);
		}
	}
}

/*
      --A--  
    F|     |B
      --G--  
    E|     |C
      --D-- o Dp

	D7 D6 D5 D4 D3 D2 D1 D0
	Dp A  B  C  D  E  F  G 
*/
const NP: u8 = 0xFF;

enum NumPos{
	VOL_1, VOL_10, VOL_100, VOL_1000,
	CUR_1, CUR_10, CUR_100, CUR_1000
}

impl NumPos {
	fn from_vol(value: u16) -> Self {
		match value {
			1000 => NumPos::VOL_1000,
			100  => NumPos::VOL_100,
			10   => NumPos::VOL_10,
			_    => NumPos::VOL_1
		}
	}
}
impl NumPos {
	fn from_cur(value: u16) -> Self {
		match value {
			1000 => NumPos::CUR_1000,
			100  => NumPos::CUR_100,
			10   => NumPos::CUR_10,
			_    => NumPos::CUR_1
		}
	}
}

const NUM_BIT_IDX_LIST: [[u8; 8]; 8] = [
	[NP,47,35,33,44,45,46,34],
	[40,55,43,41,52,53,54,42],
	[48,63,51,49,60,61,62,50],
	[56,71,59,57,68,69,70,58],
	[NP,31,27,25,28,29,30,26],
	[16,23,19,17,20,21,22,18],
	[ 8,15,11, 9,12,13,14,10],
	[ 0, 7, 3, 1, 4, 5, 6, 2],
];

const C_OUT: u8 = 66;
const C_IN: u8 = 67;
const C_SET: u8 = 65;
const C_COLON: u8 = 24;
const C_V: u8 = 32;
const C_A: u8 = 36;
const C_H: u8 = 37;
const C_W: u8 = 39;
const C_PER: u8 = 38;
const C_DEG: u8 = 64;

const FONT_7SEG: [u8; 10] = [0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B];
const NUM_CLR: u8 = 0xFF;

#[repr(u16)]
pub enum Decimal {
	None = 1,
	Ten = 10,
	Hundred = 100,
	Thousand = 1000
}

impl<CsPin, WrPin, DataPin, DLY: DelayNs> LCD1621 <CsPin, WrPin, DataPin, DLY> 
where
	CsPin: OutputPin,
	WrPin: OutputPin,
	DataPin: OutputPin
{
	fn draw_num_seg(&mut self, pos: NumPos, num: u8, dot: bool) {
		let idx_list= &NUM_BIT_IDX_LIST[pos as usize];
		let mut font;
		if num < FONT_7SEG.len() as u8 {
			font = FONT_7SEG[num as usize];
			if dot {
				font |= 0x80;
			}
		} else {
			font = 0;
		}

		for i in 0..8 {
			let idx = idx_list[i] as usize;
			if (font & (0x80 >> i)) > 0 {
				self.bit_set(idx);
			} else {
				self.bit_clr(idx);
			}
		}
	}

	fn clr_num_seg(&mut self, pos: NumPos) {
		let idx_list= &NUM_BIT_IDX_LIST[pos as usize];

		for i in 0..8 {
			let idx = idx_list[i] as usize;
			self.bit_clr(idx);
		}
	}

	fn draw_num(&mut self, mut vol: u16, dec: Decimal, is_vol: bool) {
		let mut dec = dec as u16;
		if vol >= 10_000 {
			vol /= 10;
			dec /= 10;
		}

		let mut num;
		let mut mul = 1_000;
		let f_convert = if is_vol { NumPos::from_vol } else { NumPos::from_cur };
		while mul > 0 {
			if vol >= mul {
				num = vol / mul;
				vol -= num * mul;
			} else {
				num = 0;
			}
			let dot =  if mul == dec { true } else { false };
			self.draw_num_seg(f_convert(mul), num as u8, dot);
			mul /= 10;
		}
	}

	pub fn clr_voltage(&mut self) {
		self.bit_clr(C_V as usize);
		self.clr_num_seg(NumPos::VOL_1);
		self.clr_num_seg(NumPos::VOL_10);
		self.clr_num_seg(NumPos::VOL_100);
		self.clr_num_seg(NumPos::VOL_1000);
	}

	pub fn set_voltage(&mut self, vol: u16) {//mV
		self.bit_set(C_V as usize);
		self.draw_num(vol, Decimal::Thousand, true);
	}

	pub fn set_current(&mut self, cur: u16) {//mA
		self.bit_set(C_A as usize);
		self.bit_clr(C_H as usize);
		self.bit_clr(C_W as usize);
		self.draw_num(cur, Decimal::Thousand, false);
	}

	pub fn set_watt(&mut self, watt: u16) {//mW
		self.bit_clr(C_A as usize);
		self.bit_clr(C_H as usize);
		self.bit_set(C_W as usize);
		self.draw_num(watt, Decimal::Thousand, false);
	}

	pub fn set_capacity(&mut self, cap: u16) {//mAh
		self.bit_set(C_A as usize);
		self.bit_set(C_H as usize);
		self.bit_clr(C_W as usize);
		self.draw_num(cap, Decimal::Thousand, false);
	}

	pub fn set_tip(&mut self, tip: Tip) {
		self.bit_clr(C_IN as usize);
		self.bit_clr(C_OUT as usize);
		self.bit_clr(C_SET as usize);
		
		self.bit_set(tip as usize);
	}
}

#[repr(u8)]
pub enum Tip {
	None = NUM_CLR,
	Set = C_SET,
	Out = C_OUT,
	In = C_IN
}