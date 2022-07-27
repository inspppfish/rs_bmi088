#![no_std]
#![no_main]

use defmt;
use defmt_rtt as _;
use panic_probe as _;

use stm32f4xx_hal as hal;
use hal::{
    prelude::*,
    rcc::RccExt,
};

mod bmi088_spi;
mod sensor;
mod math;
mod futaba_sbus;

use stm32f4xx_hal::gpio::*;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM3])]
mod app{
    use stm32f4xx_hal::pac::{ USART3};
    use stm32f4xx_hal::serial::{Rx, Serial};
    use systick_monotonic::fugit::Duration;
    use crate::bmi088_spi::*;
    use crate::futaba_sbus::FutabaSbus;
    use crate::sensor::*;
    use systick_monotonic::Systick;
    use super::*;

    #[shared]
    struct Shared{
        counter: u32,
        sensor: Sensor,
    }

    #[local]
    struct Local{
        imu: Bmi088,
        int_acc: PC4<Input>,
        sbus_reciver: FutabaSbus,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;


    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp: hal::pac::Peripherals = cx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(48_u32.MHz())
            .freeze();

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        let sck = gpiob.pb3.into_alternate();
        let miso = gpiob.pb4.into_alternate();
        let mosi = gpioa.pa7.into_alternate();
        let cs_acc = gpioa.pa4.into_push_pull_output();
        let cs_gyro = gpiob.pb0.into_push_pull_output();

        let mut imu = Bmi088::new(dp.SPI1,sck, miso, mosi, cs_acc, cs_gyro, clocks);
        imu.init().unwrap();


        let mut int_acc = gpioc.pc4.into_pull_up_input();
        int_acc.enable_interrupt(&mut dp.EXTI);
        int_acc.trigger_on_edge(&mut dp.EXTI, Edge::Falling);

        let mono = Systick::new(cx.core.SYST, 48_000_000);
        keep_watch::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
        show_fps::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
        get_rc_data::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();


        let rx_pin = gpioc.pc11.into_alternate();

        let mut rx:Rx<USART3> = dp.USART3.rx(rx_pin, 100_000.bps(), &clocks).unwrap();



        (
            Shared{
                counter: 0,
                sensor: Sensor::new(),
            },
            Local{
                int_acc,
                imu,
                sbus_reciver: FutabaSbus::new(rx, gpiob.pb8.into(), gpioa.pa8.into()),
            },
            init::Monotonics(mono),
        )
    }

    #[task( local = [int_acc, imu], shared = [counter, sensor])]
    fn keep_watch(mut cx: keep_watch::Context) {
        cx.local.int_acc.clear_interrupt_pending_bit();
        let imu:&mut Bmi088 = cx.local.imu;

        imu.enable_acc().unwrap();

        let mut raw_data: [u8; 6] = [0; 6];
        imu.update().unwrap();

        cx.shared.sensor.lock(|mut sensor| {
            imu.decode(&mut sensor);
        });

        cx.shared.sensor.lock(|sensor|{sensor.attitude_solution(&0.005)});

        cx.shared.counter.lock(|counter| {
            *counter = *counter + 1;
        });
        imu.silence().unwrap();

        keep_watch::spawn_after(Duration::<u64, 1, 1000>::from_ticks(4)).unwrap();
    }

    #[task(shared = [counter, sensor])]
    fn show_fps(mut cx: show_fps::Context) {
        defmt::println!("FPS: {}", cx.shared.counter.lock(|counter|{*counter}));
        // cx.shared.sensor.lock(|sensor| {
        //    defmt::println!("YAW: {}", sensor.yaw);
        //    defmt::println!("PITCH: {}", sensor.pitch);
        //    defmt::println!("ROLL: {}", sensor.roll);
        // });
        cx.shared.counter.lock(|counter|{*counter = 0});

        show_fps::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }

    #[task(local = [sbus_reciver])]
    fn get_rc_data(cx: get_rc_data::Context) {
        // defmt::println!("{}", cx.local.sbus_reciver.rx.read().unwrap());
        // get_rc_data::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }

}

