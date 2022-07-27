use cortex_m::asm::delay;
use defmt::println;
#[allow(unused_imports)]
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use stm32f4xx_hal as hal;
use hal::{
    prelude::*,
    spi::{Mode, Spi},
};
use nb::Error;

use stm32f4xx_hal::pac::SPI1;

use stm32f4xx_hal::gpio::*;
use stm32f4xx_hal::rcc::Clocks;
use crate::bmi088_spi::Bmi088ConfError::{AccConfError, AccPwrConfError, AccPwrCtrlError, AccRangeError, GyroBandwidthError, GyroCtrlError, GyroInt3Int4IoConfError, GyroInt3Int4IoMapError, GyroLpm1Error, GyroRangeError, Int1ToCtrlError, IntMapDataError};

use crate::sensor::Sensor;

pub struct Bmi088{
    spi: Spi<SPI1, (SCK, MISO, MOSI)>,
    cs_acc: PA4<Output<PushPull>>,
    cs_gyro: PB0<Output<PushPull>>,
    pub acc_raw_data: [u8; 6],
    gyro_raw_data: [u8; 6],
}

impl Bmi088 {
    pub fn new(spi1: SPI1,
               sck: SCK, miso: MISO, mosi: MOSI,
               cs_acc: PA4<Output<PushPull>>, cs_gyro: PB0<Output<PushPull>>,
               clocks: Clocks) -> Self {

        //mode for BMI088
        let spi_mode = Mode {
            polarity: hal::spi::Polarity::IdleLow,
            phase: hal::spi::Phase::CaptureOnFirstTransition,
        };

        //new from 1MHz
        let spi = Spi::new(
            spi1,
            (sck, miso, mosi),
            spi_mode,
            328125u32.Hz(),
            &clocks,
        );

        Bmi088 {
            spi,
            cs_acc,
            cs_gyro,
            acc_raw_data: [0x55, 0x55, 0x55, 0x55, 0x55, 0x55],
            gyro_raw_data: [0x55, 0x55, 0x55, 0x55, 0x55, 0x55],
        }
    }

    pub fn init(&mut self) -> Result<(), Error<()>> {
        self.silence()?;

        self.init_acc().unwrap();
        self.init_gyro().unwrap();

        defmt::println!("BMI088 init success!");
        Ok(())
    }

    pub fn update(&mut self) -> Result<(), Error<()>> {
        let mut buffer: [u8; 6] = [0; 6];
        self.acc_read_raw_data(&mut buffer).unwrap();
        self.acc_raw_data = buffer;
        self.gyro_read_raw_data(&mut buffer).unwrap();
        self.gyro_raw_data = buffer;
        Ok(())
    }

    pub fn decode(&mut self, sensor: &mut Sensor) -> () {
        const BMI088_ACCEL_3G_SEN: f32 = 0.0008974358974;
        sensor.acc_x = ((self.acc_raw_data[1]as i16)<<8 | self.acc_raw_data[0]as i16)as f32 * BMI088_ACCEL_3G_SEN;
        sensor.acc_y = ((self.acc_raw_data[3]as i16)<<8 | self.acc_raw_data[2]as i16)as f32 * BMI088_ACCEL_3G_SEN;
        sensor.acc_z = ((self.acc_raw_data[5]as i16)<<8 | self.acc_raw_data[4]as i16)as f32 * BMI088_ACCEL_3G_SEN;

        const BMI088_GYRO_2000_SEN: f32 = 0.00106526443603169529841533860381;
        sensor.gyro_x = ((self.gyro_raw_data[1]as i16)<<8 | self.gyro_raw_data[0]as i16)as f32 * BMI088_GYRO_2000_SEN;
        sensor.gyro_y = ((self.gyro_raw_data[3]as i16)<<8 | self.gyro_raw_data[2]as i16)as f32 * BMI088_GYRO_2000_SEN;
        sensor.gyro_z = ((self.gyro_raw_data[5]as i16)<<8 | self.gyro_raw_data[4]as i16)as f32 * BMI088_GYRO_2000_SEN;
    }

    fn init_acc(&mut self) -> Result<(), Error<Bmi088ConfError>>{
        //acc software reset
        self.acc_write_single_reg(0x7E, 0xB6).unwrap();

        //check communication
        //repeat the get ID instruction
        { let _res = self.acc_read_single_reg(0x00).unwrap(); }
        { let _res = self.acc_read_single_reg(0x00).unwrap(); }
        { let _res = self.acc_read_single_reg(0x00).unwrap(); }
        let res = self.acc_read_single_reg(0x00).unwrap();
        if res != 0x1E_u8 {
            defmt::panic!("Got acc ID {} ,Cannot communicate with ACCEL!", res);
        }

        let acc_conf_data_error: [([u8; 2], Bmi088ConfError) ;6] = [
            ([0x7D, 0x04,], AccPwrCtrlError),
            ([0x7C, 0x00], AccPwrConfError),
            ([0x40, (0x2 << 0x4)|(0xB << 0x0)|0x80,], AccConfError),
            ([0x41, 0x0<<0x0,], AccRangeError),
            ([0x53, (0x1 << 0x3)|(0x0 << 0x2)|(0x0<<0x1)], Int1ToCtrlError),
            ([0x58, 0x1<<0x2], IntMapDataError),
        ];

        for (instruct, error) in acc_conf_data_error {
            self.acc_write_single_reg(instruct[0], instruct[1]).unwrap();
            self.acc_write_single_reg(instruct[0], instruct[1]).unwrap();
        }
        Ok(())
    }

    fn init_gyro(&mut self) -> Result<(), Error<(Bmi088ConfError)>> {
        //acc software reset
        self.gyro_write_single_reg(0x14, 0xB6).unwrap();

        //check communication
        //repeat the get ID instruction
        { let _res = self.gyro_read_single_reg(0x00).unwrap(); }

        //todo: make this delay graceful
        let mut _a = 1000;
        while _a != 0 {
            _a -= 1;
        }

        let res = self.gyro_read_single_reg(0x00).unwrap();
        if res != 0x0F_u8 {
            defmt::panic!("Got gyro ID {} ,Cannot communicate with GYRO!", res);
        }

        let gyro_conf_data_error: [([u8; 2], Bmi088ConfError) ;6] = [
            ([0x0F, 0x00<<0x00,], GyroRangeError),
            ([0x10, 0x02 | 0x80,], GyroBandwidthError),
            ([0x11, 0x00,], GyroLpm1Error),
            ([0x15, 0x80,], GyroCtrlError),
            ([0x16, (0x0<<0x1)|(0x0<<0x0),], GyroInt3Int4IoConfError),
            ([0x18, 0x01,], GyroInt3Int4IoMapError),
        ];

        for (instruction, error) in gyro_conf_data_error {
            self.gyro_write_single_reg(instruction[0], instruction[1]).unwrap();
            self.gyro_write_single_reg(instruction[0], instruction[1]).unwrap();
        }
        Ok(())
    }

    // A method to avoid "no down trigger" in CS is silence all the CS and then pull down,
    // but i consider it of slow in this genetic func
    pub fn acc_read_single_reg(&mut self, reg: u8) -> Result<u8, Error<()>> {
        // self.silence()?;
        self.enable_acc()?;
        let mut send_msg: [u8; 3] = [0x80|reg, 0x55, 0x55];
        let res = self.spi.transfer(send_msg.as_mut_slice());
        self.silence()?;
        Ok(res.unwrap()[2])
    }

    pub fn gyro_read_single_reg(&mut self, reg: u8) -> Result<u8, Error<()>> {
        // self.silence()?;
        self.enable_gyro()?;
        let mut send_msg: [u8; 2] = [0x80|reg, 0x55];
        let res = self.spi.transfer(send_msg.as_mut_slice());
        self.silence()?;
        Ok(res.unwrap()[1])
    }

    fn acc_read_raw_data(&mut self, buffer:&mut[u8; 6]) -> Result<(), Error<()>> {
        self.enable_acc().unwrap();
        //Send instruction to get the data
        let mut send_msg: [u8; 8] =
            [0x12|0x80, 0x13|0x80, 0x14|0x80, 0x15|0x80, 0x16|0x80, 0x17|0x80, 0x55, 0x55];
        let res = self.spi.transfer(send_msg.as_mut_slice()).unwrap();
        self.silence().unwrap();

        //todo: better this
        let mut a = 2;
        for i in buffer {
            *i = res[a];
            a += 1;
        }

        Ok(())
    }

    fn gyro_read_raw_data(&mut self, buffer:&mut[u8; 6]) -> Result<(), Error<()>> {
        let mut send_msg: [u8; 7] =
            [0x02|0x80, 0x03|0x80, 0x04|0x80, 0x05|0x80, 0x06|0x80, 0x07|0x80, 0x55];
        self.enable_gyro().unwrap();
        let mut res = self.spi.transfer(send_msg.as_mut_slice()).unwrap();
        self.silence().unwrap();

        //todo: better this
        let mut a = 1;
        for i in buffer {
            *i = res[a];
            a += 1;
        }
        Ok(())
    }

    pub fn acc_write_single_reg(&mut self, reg:u8, data: u8) -> Result<(), Error<()>> {
        // self.silence().unwrap();
        self.enable_acc()?;
        let mut send_msg: [u8; 2] = [reg, data];
        let _ = self.spi.transfer(send_msg.as_mut_slice());
        self.silence()?;
        Ok(())
    }

    pub fn gyro_write_single_reg(&mut self, reg:u8, data: u8) -> Result<(), Error<()>> {
        //self.silence().unwrap();
        self.enable_gyro()?;
        let mut send_msg: [u8; 2] = [reg, data];
        let _ = self.spi.transfer(send_msg.as_mut_slice());
        self.silence()?;
        Ok(())
    }


    pub fn enable_acc(&mut self) -> Result<(), Error<()>> {
        self.cs_acc.set_low();
        self.cs_gyro.set_high();
        Ok(())
    }

    pub fn enable_gyro(&mut self) -> Result<(), Error<()>> {
        self.cs_gyro.set_low();
        self.cs_acc.set_high();
        Ok(())
    }

    pub fn silence(&mut self) -> Result<(), Error<()>> {
        self.cs_acc.set_high();
        self.cs_gyro.set_high();
        Ok(())
    }
}


pub(crate) type SCK = Pin<'B', 3, Alternate<5_u8>>;
pub(crate) type MISO = Pin<'B', 4, Alternate<5_u8>>;
pub(crate) type MOSI = Pin<'A', 7, Alternate<5_u8>>;

#[derive(Debug)]
pub enum Bmi088ConfError {
    AccPwrCtrlError,
    AccPwrConfError,
    AccConfError,
    AccRangeError,
    Int1ToCtrlError,
    IntMapDataError,
    GyroRangeError,
    GyroBandwidthError,
    GyroLpm1Error,
    GyroCtrlError,
    GyroInt3Int4IoConfError,
    GyroInt3Int4IoMapError,
}