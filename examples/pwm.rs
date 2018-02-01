#![feature(proc_macro, used, lang_items, const_fn, generators, generator_trait, asm, 
           conservative_impl_trait, universal_impl_trait, inclusive_range_syntax,
           generic_associated_types, associated_type_defaults, fmt_internals)]

#![no_std]
#![allow(dead_code)]
#![allow(mutable_transmutes)]
extern crate futures_await as futures;
extern crate cortex_m;
extern crate cortex_m_rt;
extern crate async_hal;
extern crate async_hal_stm32;
extern crate opencm;

use futures::prelude::*;
use core::cell::Cell;

use async_hal::hil::gpio::{
    AdvancedPin,
    BasicPin
};

use opencm::rcc;

use self::async_hal_stm32::hal::{
    gpio,
    timer,
};

#[async]
fn task_pwm<'f, E>(period: u16, pin: &'f E, pwm: &'f Cell<u8>) -> Result<(), ()>
    where E: BasicPin + 'f
{   
    let mut interval = timer::interval_ms(100);

    loop {
        let duty = ((period as u32) * (pwm.get() as u32) >> 8) as u16;

        interval.set_interval_us(duty);
        await_item!(interval)?;
        pin.set_high();
        
        interval.set_interval_us(period - duty);
        await_item!(interval)?;
        pin.set_low();
    }
}

// #[async]
// fn turn_on<'f, P: hil::gpio::Pin>(pin: &'f P) -> Result<(), ()> {
//    for i in 0..255 {
//        await!(pwm_pulse(pin, 3906, i))?;
//    };

//    Ok(())
// }

// #[async]
// fn turn_off<'f, P: hil::gpio::Pin>(pin: &'f P) -> Result<(), ()> {
//    for i in 0..255 {
//        await!(pwm_pulse(pin, 3906, 255 - i))?;
//    };

//    Ok(())
// }

// #[async]
// fn task<'f>(pin: &'f gpio::Pin) -> Result<(), ()> {
//     loop {
//         await!(turn_on(pin))?;
//         await!(turn_off(pin))?
//     }
// }

fn main() {
    rcc::set_clock(rcc::CrystalClock::Clock8MHz, rcc::Clock::Clock168MHz);
    timer::init();

    let blue = gpio::get_pin(3, 15);

    blue.make_output();
    blue.set_output_speed(gpio::OutputSpeed::High);

    let pwm = Cell::new(128u8);

    let _r = async_hal::reactor::run(
        task_pwm(1000, &blue, &pwm)
    );
}