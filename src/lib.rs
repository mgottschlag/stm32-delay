//! `DelayMs` and `DelayUs` implementation for STM32 MCUs.
//!
//! This crate provides an implementation of the `DelayMs` and `DelayUs` traits from `embedded-hal`
//! for various STM32 MCUs, as the corresponding HALs often only provide functionality for delays
//! based on the system timer.
//!
//! # Usage
//!
//! The `TimerDelay` type can be initialized for any type for which the `TimerExt` provided by this
//! repository is implemented and is used as follows:
//!
//! TBD
//!
//! # Limitations
//!
//! For high accuracy, the corresponding APB clock should be a multiple of 1000000. The code may
//! provide reduced accuracy for very long delays (i.e., >65k milliseconds or microseconds) as the
//! time is split into multiple shorter delays. Similarly, the code may provide reduced accuracy if
//! for `DelayMs` if the timer input clock is faster than 65536kHz, as then the limited prescaler
//! requires the whole delay loop to be repeated twice.
#![no_std]

#[cfg(not(feature = "device-selected"))]
compile_error!("A specific device needs to be selected via the appropriate feature flag.");

use embedded_hal::blocking::delay::{DelayMs, DelayUs};
#[cfg(feature = "stm32f411")]
use stm32f4xx_hal::{bb, rcc::Clocks, stm32::RCC, stm32::TIM1};

pub trait TimerExt {
    unsafe fn enable(&mut self);
    unsafe fn disable(&mut self);

    fn calc_pre(clocks: Clocks) -> (u32, u32);
    unsafe fn delay(&mut self, prescaler: u32, time: u16);
}

#[cfg(feature = "stm32f411")]
impl TimerExt for TIM1 {
    unsafe fn enable(&mut self) {
        // Enable and reset the peripheral.
        let rcc = &(*RCC::ptr());
        bb::set(&rcc.apb2enr, 0);
        bb::set(&rcc.apb2rstr, 0);
        bb::clear(&rcc.apb2rstr, 0);
        // Select down-counting mode.
        self.cr1.modify(|_, w| w.dir().set_bit());
    }

    unsafe fn disable(&mut self) {
        // Disable the peripheral.
        let rcc = &(*RCC::ptr());
        bb::set(&rcc.apb2rstr, 0);
        bb::clear(&rcc.apb2rstr, 0);
        bb::clear(&rcc.apb2enr, 0);
    }

    fn calc_pre(clocks: Clocks) -> (u32, u32) {
        let pclk_mul = if clocks.ppre2() == 1 { 1 } else { 2 };
        let freq_in = clocks.pclk2().0 * pclk_mul;
        // Higher prescalers than required are OK here, because they result in delays longer than
        // expected.
        let us_pre = (freq_in + 999999) / 1000000;
        let ms_pre = (freq_in + 999) / 1000;
        (us_pre, ms_pre)
    }

    unsafe fn delay(&mut self, prescaler: u32, time: u16) {
        // Frequencies greater than 65MHz result in prescaler values larger than 0xffff, so we need
        // to repeat the delay loop several times here.
        let repetitions = (prescaler >> 16) + 1;
        let prescaler = (prescaler & 0xffff) as u16;
        self.psc.write(|w| w.psc().bits(prescaler));
        for _ in 0..repetitions {
            // Clear the update flag.
            self.sr.write(|w| w.uif().set_bit());
            // Start the counter.
            self.cnt.write(|w| unsafe { w.cnt().bits(time) });
            self.cr1.modify(|_, w| w.cen().set_bit());
            // Wait until the counter has reached zero.
            while !self.sr.read().uif().bit_is_set() {}
            // Pause the counter.
            self.cr1.modify(|_, w| w.cen().clear_bit());
        }
    }
}

pub struct TimerDelay<T> {
    t: T,
    us_pre: u32,
    ms_pre: u32,
}

impl<T> TimerDelay<T>
where
    T: TimerExt,
{
    pub fn init(mut t: T, clocks: Clocks) -> TimerDelay<T> {
        unsafe {
            t.enable();
        };
        let (us_pre, ms_pre) = T::calc_pre(clocks);
        TimerDelay { t, us_pre, ms_pre }
    }

    pub fn free(mut self) -> T {
        unsafe {
            self.t.disable();
        }
        self.t
    }
}

impl<T> DelayMs<u8> for TimerDelay<T>
where
    T: TimerExt,
{
    fn delay_ms(&mut self, ms: u8) {
        unsafe {
            self.t.delay(self.ms_pre, ms as u16);
        }
    }
}

impl<T> DelayMs<u16> for TimerDelay<T>
where
    T: TimerExt,
{
    fn delay_ms(&mut self, ms: u16) {
        unsafe {
            self.t.delay(self.ms_pre, ms as u16);
        }
    }
}

impl<T> DelayMs<u32> for TimerDelay<T>
where
    T: TimerExt,
{
    fn delay_ms(&mut self, mut ms: u32) {
        while ms > 0xffff {
            unsafe {
                self.t.delay(self.ms_pre, 0xffff);
            }
            ms -= 0xffff;
        }
        unsafe {
            self.t.delay(self.ms_pre, ms as u16);
        }
    }
}

impl<T> DelayUs<u8> for TimerDelay<T>
where
    T: TimerExt,
{
    fn delay_us(&mut self, us: u8) {
        unsafe {
            self.t.delay(self.us_pre, us as u16);
        }
    }
}

impl<T> DelayUs<u16> for TimerDelay<T>
where
    T: TimerExt,
{
    fn delay_us(&mut self, us: u16) {
        unsafe {
            self.t.delay(self.us_pre, us as u16);
        }
    }
}

impl<T> DelayUs<u32> for TimerDelay<T>
where
    T: TimerExt,
{
    fn delay_us(&mut self, mut us: u32) {
        while us > 0xffff {
            unsafe {
                self.t.delay(self.us_pre, 0xffff);
            }
            us -= 0xffff;
        }
        unsafe {
            self.t.delay(self.us_pre, us as u16);
        }
    }
}
