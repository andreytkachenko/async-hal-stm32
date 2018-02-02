use core::marker::PhantomData;

use opencm::{
    gpio,
    rcc
};
pub use opencm::exti::TriggerMode;
pub use opencm::gpio::{
    OutputSpeed,
    OutputType,
    PuPdMode,
    PinMode
};

use async_hal::hil::gpio::{
    BasicPin,
    AdvancedPin,
    ExtiPin
};

use super::Exti;

static mut GPIO_PORT_CLIENTS: [u8; 16] = [0; 16];

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
    pub mask: u16,
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
            regs,
            data: PhantomData
        };

        pin.make_input();
        pin.set_output_speed(OutputSpeed::Low);
        pin.make_output_push_pull();
        pin.make_pulled_none();

        pin
    }
}

impl<'p> BasicPin for Pin<'p> {
    fn make_input(&self) {
        self.regs.set_pin_mode(self.pin as u16, PinMode::Input)
    }

    fn make_output(&self) {
        self.regs.set_pin_mode(self.pin as u16, PinMode::Output)
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

    fn make_pulled_up(&self) {
        self.regs.set_pin_pupd(self.pin as u16, PuPdMode::Up);
    }

    fn make_pulled_down(&self) {
        self.regs.set_pin_pupd(self.pin as u16, PuPdMode::Down);
    }

    fn make_pulled_none(&self) {
        self.regs.set_pin_pupd(self.pin as u16, PuPdMode::None);
    }

    fn set_output_speed(&self, speed: Self::OutputSpeed) {
        self.regs.set_pin_output_speed(self.pin as u16, speed);
    }

    fn make_output_open_drain(&self) {
        self.regs.set_pin_output_type(self.pin as u16, OutputType::OpenDrain);
    }

    fn make_output_push_pull(&self) {
        self.regs.set_pin_output_type(self.pin as u16, OutputType::PushPull);
    }
}

impl<'p> ExtiPin<'p> for Pin<'p> {
    type Exti = Exti<'p>;

    fn exti(&'p self) -> Option<Self::Exti> {
        Exti::new(self, TriggerMode::EitherEdge)
    }
}

impl<'p> Drop for Pin<'p> {
    fn drop(&mut self) {
        disable_pin(self.port);
    }
}