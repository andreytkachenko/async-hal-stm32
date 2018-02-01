use futures::{
    Async,
    Stream,
    Future,
    Poll
};

use core::mem;
use core::marker::PhantomData;

use hal::gpio;

use hil::serial::{
    SerialReader,
    // SerialWriter
};

pub fn get_serial<'a>(id: u8, baud_rate: u32, stop_bits: StopBits, parity: Parity) -> Uart<'a> {
    Uart::new(id, baud_rate, stop_bits, parity, false)
}

pub enum UartError {
    ParityError,
    FramingError,
    OverrunError,
    RepeatCallError,
    ResetError,
}

#[derive(Copy, Clone, Debug)]
pub enum StopBits {
    One     = 0, /* 1 stop bit */
    Half    = 1, /* 0.5 stop bits */
    Two     = 2, /* 2 stop bits */
    OneHalf = 3  /* 1.5 stop bits */
}

#[derive(Copy, Clone, Debug)]
pub enum Parity {
    None = 0,
    Odd  = 1,
    Even = 2,
}

#[derive(Copy, Clone, Debug)]
pub enum Mode {
    Receive  = 1,
    Transmit = 2,
    Duplex   = 3
}

pub enum FlowControl {
    None      = 0,
    Rts       = 1,
    Cts       = 2,
    RtsAndCts = 3
}

pub struct Uart<'a> {
    uart: &'static mut Registers,
    id: u8,
    is_usart: bool,
    _data: ::core::marker::PhantomData<&'a u32>
}

impl<'a> Uart<'a> {
    pub fn new(id: u8,
               baud_rate: u32, 
               stop_bits: StopBits, 
               parity: Parity, 
               flow_control: bool) -> Uart<'a>
    {   

        let tx = gpio::get_pin(0, 9);
        tx.set_output_speed(gpio::PinOutputSpeed::High);
        tx.set_pull(gpio::PinPullUpMode::Up);
        tx.select_peripheral(gpio::function::USART1);
        
        let rx = gpio::get_pin(0, 10);
        rx.set_output_speed(gpio::PinOutputSpeed::High);
        rx.set_pull(gpio::PinPullUpMode::Up);
        rx.select_peripheral(gpio::function::USART1);

        rcc::enable(match id {
            0 => rcc::Peripheral::USART1,
            1 => rcc::Peripheral::USART2,
            2 => rcc::Peripheral::USART3,
            3 => rcc::Peripheral::UART4,
            4 => rcc::Peripheral::UART5,
            5 => rcc::Peripheral::USART6,
            6 => rcc::Peripheral::UART7,
            7 => rcc::Peripheral::UART8,
            _ => unreachable!()
        });
        
        let mut dev = Uart {
            id,
            uart: unsafe { mem::transmute(UART[id as usize]) },
            is_usart: false,
            _data: ::core::marker::PhantomData
        };

        dev.set_baudrate(baud_rate);
        dev.set_parity(parity);
        dev.set_stopbits(stop_bits);
        dev.set_flow_control(if flow_control {FlowControl::RtsAndCts} else {FlowControl::None});
        dev.enable_interrupts();
        dev.enable_error_interrupts();
        dev.set_databits(8);
        dev.set_mode(Mode::Duplex);
        dev.enable();

        if id != 6 && id != 7 {
            nvic::enable(match id {
                0 => nvic::NvicIdx::USART1,
                1 => nvic::NvicIdx::USART2,
                2 => nvic::NvicIdx::USART3,
                3 => nvic::NvicIdx::UART4,
                4 => nvic::NvicIdx::UART5,
                5 => nvic::NvicIdx::USART6,
                _ => unreachable!()
            });
        }

        dev
    }
}

impl<'a> hil::Uart for Uart {
    fn set_baudrate(&self, baud: u32) {
        let clock = if self.id == 1 || self.id == 6 {
            rcc::apb2_frequency()
        } else {
            rcc::apb1_frequency()
        };

        self.uart.brr.set(((clock << 1) + baud) / (baud << 1));
    }
    
    fn set_databits(&mut self, bits: u32) {
        if bits == 8 {
            self.uart.cr1 &= !flags::cr1::M; /* 8 data bits */
        } else {
            self.uart.cr1 |= flags::cr1::M; /* 9 data bits */
        }
    }
    
    fn set_stopbits(&self, stopbits: StopBits) {
        let mut reg32 = self.uart.cr2.get();
        reg32 = (reg32 & !flags::cr2::STOPBITS_MASK) | (stopbits as u32);
        self.uart.cr2.set(reg32);
    }
    
    fn set_parity(&self, parity: Parity) {
        let mut reg32 = self.uart.cr1.get();
        reg32 = (reg32 & !flags::cr1::PARITY_MASK) | (parity as u32);
        self.uart.cr1.set(reg32);
    }
    
    fn set_mode(&self, mode: Mode) {
        let mut reg32 = self.uart.cr1.get();
        reg32 = (reg32 & !flags::cr1::MODE_MASK) | (mode as u32);
        self.uart.cr1.set(reg32);
    }
    
    fn set_flow_control(&self, flowcontrol: FlowControl) {
        let mut reg32 = self.uart.cr3.get();
        reg32 = (reg32 & !flags::cr3::FLOWCONTROL_MASK) | (flowcontrol as u32);
        self.uart.cr3.set(reg32);
    }
    
    fn enable(&mut self) {
        self.uart.cr1 |= flags::cr1::UE;
    }
    
    fn disable(&mut self) {
        self.uart.cr1 &= !flags::cr1::UE;
    }
    
    fn send(&self, data: u16) {
        self.uart.dr.set((data as u32) & flags::dr::MASK);
    }

    fn recv(&self) -> Result<u16, UartError> {
        Ok((self.uart.dr & flags::dr::MASK) as u16)
    }
    
    fn wait_send_ready(&self) {
        while (self.uart.sr & flags::sr::TXE) == 0 {};
    }
    
    fn wait_recv_ready(&self) {
        while (self.uart.sr & flags::sr::RXNE) == 0 {};
    }
    
    fn send_blocking(&self, data: u16) {
        self.wait_send_ready();
	    self.send(data);
    }
    
    fn recv_blocking(&self) -> Result<u16, UartError> {
        self.wait_recv_ready();
	    self.recv()
    }

    fn enable_interrupts(&mut self) {
        self.uart.cr1 |= flags::cr1::RXNEIE | flags::cr1::TCIE;
    }

    fn disable_interrupts(&mut self) {

    }

    fn enable_error_interrupts(&mut self) {
        // self.uart.cr1 |= flags::cr1::PEIE;
        // self.uart.cr2 |= flags::cr2::LBDIE;
        self.uart.cr3 |= flags::cr3::EIE;
    }

    fn disable_error_interrupts(&mut self) {
        // self.uart.cr1 &= !flags::cr1::PEIE;
        // self.uart.cr2 &= !flags::cr2::LBDIE;
        self.uart.cr3 &= !flags::cr3::EIE;
    }
}

pub struct RecvIterator<'a> {
    uart: &'a Uart<'a>
}

impl<'a> Iterator for RecvIterator<'a> {
    type Item = Result<u16, UartError>;

    fn next(&mut self) -> Option<Result<u16, UartError>> {
        Some(self.uart.recv_blocking())
    }
}

pub struct UartReadFuture<'a> {
    usart_id: usize,
    _data: PhantomData<&'a u32>
}

impl<'a> Future for UartReadFuture<'a> {
    type Item = u16;
    type Error = UartError;

    fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
        match unsafe {USART_VALUE[self.usart_id]} {
            Some(value) => {
                unsafe {USART_VALUE[self.usart_id] = None};

                Ok(Async::Ready(value))
            },
            None => Ok(Async::NotReady)
        }
    }
}

pub struct UartReadStream<'a> {
    usart_id: usize,
    _data: PhantomData<&'a u32>
}

impl<'a> Stream for UartReadStream<'a> {
    type Item = u16;
    type Error = UartError;

    fn poll(&mut self) -> Poll<Option<Self::Item>, Self::Error> {
        match unsafe {USART_VALUE[self.usart_id]} {
            Some(value) => {
                unsafe {USART_VALUE[self.usart_id] = None};

                Ok(Async::Ready(Some(value)))
            },
            None => Ok(Async::NotReady)
        }
    }
}

pub struct UartReadingDoneFuture<'a> {
    usart_id: usize,
    _data: PhantomData<&'a u32>
}

impl<'a> Future for UartReadingDoneFuture<'a> {
    type Item = &'a mut [u16];
    type Error = UartError;
    fn poll(&mut self) -> Poll<Self::Item, Self::Error> {
        let _pt = unsafe {&mut USART_BUFFER[self.usart_id]};

        Ok(Async::NotReady)
    }
}

impl<'a> SerialReader<'a, u16> for Uart<'a> {
    type Error = UartError;
    type ReadIterator = RecvIterator<'a>;
    type ReadFuture = UartReadFuture<'a>;
    type ReadStream = UartReadStream<'a>;
    type ReadingDoneFuture = UartReadingDoneFuture<'a>;

    fn read(&self) -> Self::ReadFuture {
        UartReadFuture {
            usart_id: self.id as usize,
            _data: PhantomData
        }
    }
    fn read_exact(&self, _buff: &'a mut [u16]) -> Self::ReadingDoneFuture {
        UartReadingDoneFuture {
            usart_id: self.id as usize,
            _data: PhantomData
        }
    }
    fn read_stream(&self) -> Self::ReadStream {
        UartReadStream {
            usart_id: self.id as usize,
            _data: PhantomData
        }
    }

    fn read_sync(&self) -> Result<u16, Self::Error> {
        self.recv_blocking()
    }

    fn read_exact_sync<'b>(&'a self, buff: &'b mut [u16]) -> Result<&'b mut [u16], Self::Error> {
        for word in buff.iter_mut() {
            *word = self.recv_blocking()?;
        }

        Ok(buff)
    }

    fn read_iter(&'a self) -> Self::ReadIterator {
        RecvIterator { uart: self }
    }
}

fn recv(id: u8) -> u16 {
    let usart: &Registers = unsafe { mem::transmute(UART[id as usize]) };

    (usart.dr & flags::dr::MASK) as u16
}

fn usart_check(id: u8, flag: u32) -> bool {
    let usart: &Registers = unsafe { mem::transmute(UART[id as usize]) };

    (usart.sr & flag) != 0
} 

fn usart_reset(id: u8, flag: u32) {
    let usart: &mut Registers = unsafe { mem::transmute(UART[id as usize]) };

    usart.sr &= !flag;
}

static mut USART_VALUE: [Option<u16>; 6] = [None; 6];
static mut USART_BUFFER: [Option<&mut [u16]>; 6] = [None, None, None, None, None, None];
static mut USART_BUFFER_READY: [bool; 6] = [false; 6];
static mut USART_BUFFER_CURSOR: [usize; 6] = [0; 6];

pub extern "C" fn usart1_handler() {
    ::cortex_m::interrupt::free(|_| unsafe {
        if usart_check(0, flags::sr::RXNE) {
            usart_reset(0, flags::sr::RXNE);

            USART_VALUE[0] = Some(recv(0));
            
            ::event::set_action_flag();
        }

        let usart: &mut Registers = mem::transmute(UART[0]);

        usart.sr.set(0);
        // if (USART1->SR & USART_SR_RXNE) {
        //     // Сбрасываем флаг прерывания
        //     USART1->SR&=~USART_SR_RXNE; 


        // match USART_BUFFER[0] {
        //     Some(ref mut buff) => {
        //         let cursor = &mut USART_BUFFER_CURSOR[0];

        //         buff[*cursor] = recv(0);
        //         *cursor += 1;

        //         if *cursor == buff.len() {
        //             ::event::set_action_flag();
        //         }
        //     },
        //     None => {
        //         USART_VALUE[0] = Some(recv(0));
        //         ::event::set_action_flag();
        //     }
        // }
    });
}