use futures::{
    Poll,
    Async,
    Stream
};

use opencm::exti::{
    TriggerMode,
    EdgeType
};

use opencm::{
    syscfg,
    nvic,
    rcc,
    exti
};

use async_hal::hil;
use async_hal::hil::gpio::BasicPin;

use super::Pin;

static mut EXTI_LINE_FLAG: [bool; 16] = [false; 16];
static mut EXTI_LINE_BUSY: u16 = 0;

pub struct Exti<'p> {
    edge: TriggerMode,
    pin: &'p Pin<'p>
}

impl<'p> Exti<'p> {
    pub fn new(pin: &'p Pin, edge: TriggerMode) -> Option<Exti<'p>> {
        if unsafe {EXTI_LINE_BUSY & pin.mask} == 0 {
            rcc::enable(rcc::Peripheral::SYSCFG);
            exti::enable(pin.mask as u32);
            exti::set_trigger(pin.mask as u32, edge);

            let irq = match pin.pin {
                0 => nvic::NvicIdx::EXTI0,
                1 => nvic::NvicIdx::EXTI1,
                2 => nvic::NvicIdx::EXTI2,
                3 => nvic::NvicIdx::EXTI3,
                4 => nvic::NvicIdx::EXTI4,
                5...9 => nvic::NvicIdx::EXTI9_5,
                10...15 => nvic::NvicIdx::EXTI15_10,
                _ => panic!("wrong pin number")
            };

            syscfg::select_exti_source(pin.pin as u16, pin.port as u16);
            nvic::enable(irq);

            unsafe {
                EXTI_LINE_BUSY |= pin.mask;
            }

            Some(Exti {
                edge,
                pin
            })
        } else {
            None
        }
    }
}

impl<'p> Drop for Exti<'p> {
    fn drop(&mut self) {
        unsafe {
            EXTI_LINE_BUSY &= !self.pin.mask;
        }
    }
}

impl<'p> hil::gpio::Exti for Exti<'p> {
    type EdgeType = EdgeType;

    fn trigger_on_rising(&self) {
        exti::trigger_on_rising(self.pin.mask as u32);
    }

    fn trigger_on_falling(&self) {
        exti::trigger_on_falling(self.pin.mask as u32);
    }

    fn trigger_on_both(&self) {
        exti::trigger_on_both(self.pin.mask as u32);
    }

    fn trigger_off(&self) {
        exti::trigger_off(self.pin.mask as u32);
    }
}

impl<'p> Stream for Exti<'p> {
    type Item = EdgeType;
    type Error = ();

    fn poll(&mut self) -> Poll<Option<Self::Item>, Self::Error> {
        if unsafe {EXTI_LINE_FLAG[self.pin.pin as usize]} {
            unsafe {EXTI_LINE_FLAG[self.pin.pin as usize] = false};

            match self.edge {
                TriggerMode::RisingEdge  => Ok(Async::Ready(Some(EdgeType::Rising))),
                TriggerMode::FallingEdge => Ok(Async::Ready(Some(EdgeType::Falling))),
                TriggerMode::EitherEdge  => {
                    Ok(Async::Ready(Some(if self.pin.read() {EdgeType::Rising} else {EdgeType::Falling})))
                }
            }
        } else {
            Ok(Async::NotReady)
        }
    }
}

macro_rules! make_handler {
    ($name: ident, $line: expr, $event: ident) => {
        pub extern "C" fn $name() {
            let mask = 1u32 << $line;

            if exti::get_flag_status(mask) {
                exti::reset_flag(mask);

                unsafe {EXTI_LINE_FLAG[$line as usize] = true}; 

                ::async_hal::reactor::set_action_flag();
            }
        }
    }
}

make_handler!(exti0_handler, 0, Exti0);
make_handler!(exti1_handler, 1, Exti1);
make_handler!(exti2_handler, 2, Exti2);
make_handler!(exti3_handler, 3, Exti3);
make_handler!(exti4_handler, 4, Exti3);

pub extern "C" fn exti9_5_handler() {
    for line in 5..=9 {
        let mask = 1u32 << line;

        if exti::get_flag_status(mask) {
            exti::reset_flag(mask);

            unsafe {EXTI_LINE_FLAG[line as usize] = true}; 

            ::async_hal::reactor::set_action_flag();
        }
    }
}

pub extern "C" fn exti15_10_handler() {
    for line in 10..=15 {
        let mask = 1 << line;
        
        if exti::get_flag_status(mask) {
            exti::reset_flag(mask);

            unsafe {EXTI_LINE_FLAG[line as usize] = true};

            ::async_hal::reactor::set_action_flag();
        }
    }
}