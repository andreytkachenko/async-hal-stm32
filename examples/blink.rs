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

use futures::prelude::*;

use async_hal::hil::gpio::BasicPin;

use self::async_hal_stm32::hal::{
    gpio,
    timer,
};

#[async]
fn task_blink<'f>(period_ms: u16, led: &'f gpio::Pin) -> Result<(), ()> {
    #[async]
    for _ in timer::interval_ms(period_ms) {
        led.toggle();
    }

    Ok(())
}

fn main() {
    timer::init();

    let blue = gpio::get_pin(3, 15);

    blue.make_output();

    let _r = async_hal::reactor::run(task_blink(1000, &blue));
}