#[async]
fn task_pwm<'f, E>(period: u16, pin: &'f E, pwm: &'f Cell<u8>) -> Result<(), ()>
    where E: hil::gpio::Pin + 'f
{   
    let mut interval = timer::interval_ms(100);

    loop {
        let duty = ((period as u32) * (pwm.get() as u32) >> 8) as u16;

        interval.set_interval_us(duty);
        await_item!(interval)?;
        pin.set_high();
        
        interval.set_interval_us(period - duty);
        await_item!(interval)?;
        pin.set_low();
    }
}

#[async]
fn echo<'f, E>(echo: &'f E) -> Result<u32, ()>
    where E: hil::gpio::Pin + 'f
{
    systick::reset();
    let mut exti = echo.exti(hil::exti::InterruptMode::RisingEdge).unwrap();
    
    await_item!(exti)?.ok_or(())?;

    exti.capture_both();

    let time = systick::ticks();

    await_item!(exti)?;

    let delta = time - systick::ticks();

    Ok(((delta >> 1) * 10_000) / 30302)
}

#[async]
fn task3<'f, E>(echo_pin: &'f E, pwm: &'f Cell<u8>) -> Result<(), ()>
    where E: hil::gpio::Pin + 'f
{
    loop {
        let dist = await!(echo(echo_pin))?;

        pwm.set(((dist << 8) / 5000) as u8);
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