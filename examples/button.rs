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

use self::async_hal_stm32::hal::{ gpio, timer };

#[async]
fn button_led<'f>(button: &'f gpio::Pin, led: &'f gpio::Pin) -> Result<(), ()>{
    let mut exti = button.exti().unwrap();
    exti.trigger_on_rising();

    led.set_high();

    #[async]
    for _ in exti {
        led.toggle();
    }

    Ok(())
}

#[async]
fn button_led_jitter<'f>(button: &'f gpio::Pin, led: &'f gpio::Pin) -> Result<(), ()> {
    let mut exti = button.exti().unwrap();
    exti.trigger_on_rising();
    led.set_high();

    loop {
        await_item!(exti)?;
        exti.trigger_off();
        led.toggle();
        await!(timer::timeout_ms(100))?;
        exti.trigger_on_rising();
    }
}

fn main() {
    timer::init();
    let button = gpio::get_pin(0, 0);
    let blue = gpio::get_pin(3, 15);

    button.make_input();
    blue.make_output();

    let _r = async_hal::reactor::run(button_led_jitter(&button, &blue));
}