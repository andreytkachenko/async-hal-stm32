use cortex_m;

use futures::{
    Stream,
    Future,
    Poll,
    Async   
};
use opencm::{
    rcc,
    nvic,
    timer
};

pub struct Channel {
    fired: bool,
    timer: u8
}

static mut CHANNELS: [bool; 14] = [false; 14];

pub fn init() {
    rcc::enable(rcc::Peripheral::TIM6);
    timer::enable_update_event(timer::TIM6);
    timer::enable_update_irq(timer::TIM6);
    nvic::enable(nvic::NvicIdx::TIM6_DAC);
}

pub fn start(psc: u16, period: u16, continuous: bool) {
    unsafe{CHANNELS[6] = false};

    timer::enable_update_irq(timer::TIM6);

    if continuous {
        timer::continuous_mode(timer::TIM6);
    } else {
        timer::one_shot_mode(timer::TIM6);
    }

    timer::set_prescaler(timer::TIM6, psc as u32);
    timer::set_period(timer::TIM6, period as u32);
    timer::enable_preload(timer::TIM6);
    timer::enable_counter(timer::TIM6);
}

pub fn update(psc: u16, period: u16, continuous: bool) {
    unsafe{CHANNELS[6] = false};

    if continuous {
        timer::continuous_mode(timer::TIM6);
    } else {
        timer::one_shot_mode(timer::TIM6);
    }

    timer::set_prescaler(timer::TIM6, psc as u32);
    timer::set_period(timer::TIM6, period as u32);
    timer::enable_counter(timer::TIM6);
}

pub fn current() -> u32 {
    timer::get_counter(timer::TIM6)
}

pub fn timeout_ms(timeout: u16) -> TimeoutFuture {
    TimeoutFuture::new((rcc::apb2_ms_psc() >> 2) as u16 - 1, (timeout << 3) - 1)
}

pub fn interval_ms(interval: u16) -> IntervalStream {
    IntervalStream::new((rcc::apb2_ms_psc() >> 2) as u16 - 1, ((interval << 3) << 1) - 1)
}

pub fn timeout_us(timeout: u16) -> TimeoutFuture {
    TimeoutFuture::new(rcc::apb2_us_psc() as u16 - 1, (timeout << 1) - 1)
}

pub fn interval_us(interval: u16) -> IntervalStream {
    IntervalStream::new((rcc::apb2_us_psc() as u16) - 1, (interval << 1) - 1)
}
    
pub struct TimeoutFuture {
    psc: u16,
    period: u16
}

impl TimeoutFuture {
    pub fn new(psc: u16, period: u16) -> TimeoutFuture {
        start(psc, period, false);
        
        TimeoutFuture { psc, period }
    } 
}

impl Future for TimeoutFuture {
    type Item = ();
    type Error = ();

    fn poll(&mut self) -> Poll<(), ()> {
        if unsafe{CHANNELS[6]} {
            unsafe{CHANNELS[6] = false};

            Ok(Async::Ready(()))
        } else {
            Ok(Async::NotReady)
        }
    }
}

pub struct IntervalStream {
    psc: u16,
    interval: u16
}

impl IntervalStream {
    pub fn new(psc: u16, interval: u16) -> IntervalStream {
        start(psc, interval, true);
        
        IntervalStream {
            psc, interval
        } 
    } 

    pub fn set_interval_us(&self, interval: u16) {
        update((rcc::apb2_us_psc() as u16) - 1, (interval << 1) - 1, true);
    }

    pub fn set_interval_ms(&self, interval: u16) {
        update(((rcc::apb2_ms_psc() >> 2) as u16) - 1, (interval << 3) - 1, true);
    }
}

impl Stream for IntervalStream {
    type Item = ();
    type Error = ();

    fn poll(&mut self) -> Poll<Option<()>, ()> {
        if unsafe{CHANNELS[6]} {
            unsafe{CHANNELS[6] = false};

            Ok(Async::Ready(Some(())))
        } else {
            Ok(Async::NotReady)
        }
    }
}

/** Interrupt handlers **/

pub extern "C" fn tim6_handler() {
    cortex_m::interrupt::free(|_| {
        if timer::is_update_flag_checked(timer::TIM6) {
            timer::clear_update_flag(timer::TIM6);

            unsafe{CHANNELS[6] = true};

            ::async_hal::reactor::set_action_flag();
        }
    });
}

pub extern "C" fn tim7_handler() {
    cortex_m::interrupt::free(|_| {
       if timer::is_update_flag_checked(timer::TIM7) {
            timer::clear_update_flag(timer::TIM7);

            unsafe{CHANNELS[6] = true};

            ::async_hal::reactor::set_action_flag();
        }
    });
}