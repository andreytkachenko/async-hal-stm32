use cortex_m::interrupt;
use cortex_m::peripheral::{
    SYST,
    SystClkSource
};

pub fn init() {
    interrupt::free(|cs| {
        let syst = SYST.borrow(cs);

        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload(0x00FF_FFFF);
        syst.enable_counter();
    })
}

pub fn reset() {
    interrupt::free(|cs|SYST.borrow(cs).clear_current());
}

pub fn ticks() -> u32 {
    interrupt::free(|cs|SYST.borrow(cs).get_current() >> 3)
}