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

use async_hal::hil::gpio::{
    BasicPin,
    ExtiPin,
    Exti
};

use self::async_hal_stm32::hal::gpio;

#[async]
fn button_led<'f>(button: &'f gpio::Pin, led: &'f gpio::Pin) -> Result<(), ()>{
    let exti = button.exti().unwrap();
    let mut exti_stream = exti.stream();

    led.set_high();

    #[async]
    for _ in exti_stream {
        led.toggle();
    }

    Ok(())
}

fn main() {
    let button = gpio::get_pin(0, 0);
    let blue = gpio::get_pin(3, 15);

    button.make_input();
    blue.make_output();

    let _r = async_hal::reactor::run(button_led(&button, &blue));
}