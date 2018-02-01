use core::marker::PhantomData;
use opencm::{
    syscfg,
    nvic,
    gpio,
    rcc,
    exti
};
use futures::{
    Poll,
    Async,
    Stream
};

pub use opencm::gpio::{
    OutputSpeed,
    OutputType,
    PuPdMode,
    PinMode
};

pub use opencm::exti::{
    TriggerMode,
    EdgeType
};

use async_hal::hil;
use async_hal::hil::gpio::{
    BasicPin,
    AdvancedPin,
    ExtiPin
};

static mut EXTI_LINE_FLAG: [bool; 16] = [false; 16];
static mut GPIO_PORT_CLIENTS: [u8; 16] = [0; 16];
static mut EXTI_BUSY: u16 = 0;

pub fn enable_gpio_port(port: u8) {
	rcc::enable(match port {
		0 => rcc::Peripheral::GPIOA,
		1 => rcc::Peripheral::GPIOB,
		2 => rcc::Peripheral::GPIOC,
		3 => rcc::Peripheral::GPIOD,
		4 => rcc::Peripheral::GPIOE,
		5 => rcc::Peripheral::GPIOF,
		6 => rcc::Peripheral::GPIOG,
		7 => rcc::Peripheral::GPIOH,
		8 => rcc::Peripheral::GPIOI,
		9 => rcc::Peripheral::GPIOJ,
		10 => rcc::Peripheral::GPIOK,
		_ => panic!("wrong gpio port")
	});
}

pub fn disable_gpio_port(port: u8) {
	rcc::disable(match port {
		0 => rcc::Peripheral::GPIOA,
		1 => rcc::Peripheral::GPIOB,
		2 => rcc::Peripheral::GPIOC,
		3 => rcc::Peripheral::GPIOD,
		4 => rcc::Peripheral::GPIOE,
		5 => rcc::Peripheral::GPIOF,
		6 => rcc::Peripheral::GPIOG,
		7 => rcc::Peripheral::GPIOH,
		8 => rcc::Peripheral::GPIOI,
		9 => rcc::Peripheral::GPIOJ,
		10 => rcc::Peripheral::GPIOK,
		_ => panic!("wrong gpio port")
	});
}

fn enable_pin(port_num: u8) {
    if unsafe {GPIO_PORT_CLIENTS[port_num as usize]} == 0 {
        enable_gpio_port(port_num);
    }

    unsafe {GPIO_PORT_CLIENTS[port_num as usize] += 1};
}

fn disable_pin(port_num: u8) {
    if unsafe {GPIO_PORT_CLIENTS[port_num as usize]} == 1 {
        disable_gpio_port(port_num);
    }

    unsafe {GPIO_PORT_CLIENTS[port_num as usize] -= 1};
}

pub fn get_pin<'p>(port: u8, pin: u8) -> Pin<'p> {
    Pin::new(port, pin)
}

pub struct Pin<'p> {
    pub port: u8,
    pub pin:  u8,
    mask: u16,
    regs: gpio::Gpio,
    data: PhantomData<&'p u8>
}

impl<'p> Pin<'p> {
    fn new(port: u8, pin: u8) -> Pin<'p> {
        let regs = gpio::get_port_by_index(port as u16);

        enable_pin(port);

        let pin = Pin {
            port,
            pin,
            mask: 1u16 << pin,
            regs: regs,
            data: PhantomData
        };

        pin.make_input();
        pin.set_output_speed(OutputSpeed::Low);
        pin.make_push_pull_output();
        pin.make_pull_none();

        pin
    }
}


impl<'p> BasicPin for Pin<'p> {
    fn make_input(&self) {
        self.regs.set_pin_mode(self.mask, PinMode::Input)
    }

    fn make_output(&self) {
        self.regs.set_pin_mode(self.mask, PinMode::Output)
    }

    fn set_high(&self) {
        self.regs.set_high(self.mask);
    }

    fn set_low(&self) {
        self.regs.set_low(self.mask);
    }

    fn toggle(&self) {
        self.regs.toggle(self.mask);
    }

    fn read(&self) -> bool {
        self.regs.read_pins(self.mask) != 0
    }
}

impl<'p> AdvancedPin for Pin<'p> {
    type OutputSpeed = OutputSpeed;

    fn make_pull_up(&self) {
        self.regs.set_pin_pupd(self.pin as u16, PuPdMode::Up);
    }

    fn make_pull_down(&self) {
        self.regs.set_pin_pupd(self.pin as u16, PuPdMode::Down);
    }

    fn make_pull_none(&self) {
        self.regs.set_pin_pupd(self.pin as u16, PuPdMode::None);
    }

    fn set_output_speed(&self, speed: Self::OutputSpeed) {
        self.regs.set_pin_output_speed(self.pin as u16, speed);
    }

    fn make_open_drain_output(&self) {
        self.regs.set_pin_output_type(self.pin as u16, OutputType::OpenDrain);
    }

    fn make_push_pull_output(&self) {
        self.regs.set_pin_output_type(self.pin as u16, OutputType::PushPull);
    }
}

impl<'p> ExtiPin<'p> for Pin<'p> {
    type Exti = Exti<'p>;

    fn exti(&'p self) -> Option<Self::Exti> {
         if unsafe {EXTI_BUSY & self.mask} == 0 {
            Some(Exti::new(self, TriggerMode::EitherEdge))
        } else {
            None
        }
    }
}

impl<'p> Drop for Pin<'p> {
    fn drop(&mut self) {
        disable_pin(self.port);
    }
}

pub struct Exti<'p> {
    edge: TriggerMode,
    pin: &'p Pin<'p>
}

impl<'p> Exti<'p> {
    fn new(pin: &'p Pin, edge: TriggerMode) -> Exti<'p> {
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
            EXTI_BUSY |= pin.mask;
        }

        Exti {
            edge,
            pin
        }
    }
}

impl<'p> Drop for Exti<'p> {
    fn drop(&mut self) {
        unsafe {
            EXTI_BUSY &= !self.pin.mask;
        }
    }
}

impl<'p> hil::gpio::Exti<'p> for Exti<'p> {
    type EdgeType = EdgeType;
    type Stream = ExtiStream<'p>;
    
    fn trigger_on_rising(&self) {
        exti::trigger_on_rising(self.pin.mask as u32);
    }

    fn trigger_on_falling(&self) {
        exti::trigger_on_falling(self.pin.mask as u32);
    }

    fn trigger_on_both(&self) {
        exti::trigger_on_both(self.pin.mask as u32);
    }

    fn stream(&'p self) -> Self::Stream {
        ExtiStream { exti: self }
    }
}

pub struct ExtiStream<'p> {
    exti: &'p Exti<'p>
}

impl<'p> Stream for ExtiStream<'p> {
    type Item = EdgeType;
    type Error = ();

    fn poll(&mut self) -> Poll<Option<Self::Item>, Self::Error> {
        if unsafe {EXTI_LINE_FLAG[self.exti.pin.pin as usize]} {
            unsafe {EXTI_LINE_FLAG[self.exti.pin.pin as usize] = false};

            match self.exti.edge {
                TriggerMode::RisingEdge  => Ok(Async::Ready(Some(EdgeType::Rising))),
                TriggerMode::FallingEdge => Ok(Async::Ready(Some(EdgeType::Falling))),
                TriggerMode::EitherEdge  => {
                    Ok(Async::Ready(Some(if self.exti.pin.read() {EdgeType::Rising} else {EdgeType::Falling})))
                }
            }
        } else {
            Ok(Async::NotReady)
        }
    }
}

/** Interrups handlers **/

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