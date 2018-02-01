#[async]
fn read_dht11<'f>(pin: &'f gpio::Pin) -> Result<(u16, u16), ()> {
    systick::reset();

    let mut data: [u8; 5] = [0; 5];

    pin.make_output();

    pin.set_high();
    await!(timer::timeout_ms(500))?;

    pin.set_low();
    await!(timer::timeout_ms(20))?;

    pin.set_high();

    let mut exti = pin.exti(hil::exti::InterruptMode::EitherEdge).unwrap();
    let mut stream = exti.skip(3);

    pin.make_input();

    let mut time = systick::ticks();
    let mut counter: u16 = 0;
    
    #[async]
    for _ in stream {
        if counter > 0 && (counter & 0b1 == 0) {
            let new_time = systick::ticks();

            if time - new_time > 100 {
                data[(counter >> 3) as usize] |= 1 << (counter & 0b111);
            }

            time = new_time;
        }

        counter += 1;
    }

    if data[0] + data[1] + data[2] + data[3] != data[4] {
        Err(())
    } else {
        Ok((
            ((data[0] as u16) << 8) | (data[1] as u16),
            ((data[2] as u16) << 8) | (data[3] as u16),
        ))
    }
}

fn main() {
    rcc::set_clock(rcc::CrystalClock::Clock8MHz, rcc::Clock::Clock168MHz);
    timer::init();

    let blue = gpio::get_pin(3, 15);

    blue.make_output();
    blue.set_output_speed(gpio::PinOutputSpeed::High);

    let _r = async_hal::reactor::execute(
        task_blink(1000, &blue)
    );
}

//exception!(SYS_TICK, sys_tick);
//
//#[inline]
//fn sys_tick() {
//    system::UPTIME += 1;
//}