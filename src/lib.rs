//! A platform agnostic driver to interface with the LSM303AGR (accelerometer + compass)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
//!
//! # Examples
//!

//#![deny(missing_docs)]
//#![deny(warnings)]
#![no_std]
#![feature(uniform_paths)]
#![feature(proc_macro_hygiene)]

use seq_macro::seq;

extern crate cast;
extern crate embedded_hal as hal;
extern crate generic_array;
use u8bits::u8bits;
use surjective_enum::From;

use core::mem;
use core::default::Default;

use cast::u16;
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};
use hal::blocking::i2c::{Write, WriteRead};

use SlaveAddressEnum::*;
use Register::*;


pub trait RegBytes: Default + AsRef<[u8]> + AsMut<[u8]> {
    /// Slave Address
    const SAD: u8; 
}

macro_rules! reg_bytes { 
    { $(#[$attr:meta])* $name:ident, $length:expr, $reg:ident, $sad:ident;
      $($t:tt)* } => {
        $(#[$attr])*
        pub struct $name( [u8;$length]);
        impl Default for $name {
            fn default()-> Self {
                $name( seq!(N in 0..=2 {
                    [$reg as u8, #(0,)*]
                }))
            }
        }
        impl AsRef<[u8]> for $name {
            #[inline] fn as_ref(&self) -> &[u8] { &self.0[..] }
        }
        impl AsMut<[u8]> for $name {
            #[inline] fn as_mut(&mut self) -> &mut [u8] { &mut self.0[..] }
        }
        impl RegBytes for $name { const SAD: u8 = $sad as u8; }
        impl $name {
            u8bits! { $($t)* }
        }
    };
}


/// LSM303AGR driver
pub struct Lsm303agr<I2C> {
    i2c: I2C,
    /// conversion for acceleration measurements
    acc_shift: u8,
    acc_scale: i16,
}

impl<I2C, E> Lsm303agr<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Creates a new driver from a I2C peripheral
    pub fn new(i2c: I2C) -> Result<Self, E> {
        let lsm303agr = Lsm303agr { i2c, acc_scale:0, acc_shift:0};
        Ok(lsm303agr)
    }

    pub fn set_accel_mode(&mut self, m: AccelMode) -> Result<(), E> {
        let lpen: u8 = match m { AccelMode::LowPower(_,_) => 0b0000_1000, _ => 0 };
        let (odr,fs,scale,shift): (u8,u8,i16,u8) = match m {
            AccelMode::PowerDown => (0,0,0,0),
            AccelMode::LowPower( odr, sens) // 8 bit
                => (odr as u8, sens as u8, sens.scale(), 8),
            AccelMode::Normal( odr, sens)  // 10 bit
                => (odr as u8, sens as u8, sens.scale() * 4, 6),
            AccelMode::HighResolution(odr, sens) // 12 bit
                => (odr as u8, sens as u8, sens.scale() * 16, 4),
        };
        let hr: u8 = match m { AccelMode::HighResolution(_,_) => 0b0000_1000, _ => 0 };
        self.acc_scale = scale; self.acc_shift = shift;
        self.modify_register(ACCEL, CTRL_REG1_A, |r| { r & !0b1111_1000 | lpen | (odr<<4) })?;
        self.modify_register(ACCEL, CTRL_REG4_A, |r| { r & !0b0011_1000 | hr | (fs<<4) })
    }
    
    /// Accelerometer measurements in milli gravity g
    pub fn accel(&mut self) -> Result<I16x3, E> {
        let buffer: GenericArray<u8, U6> = self.read_registers( ACCEL, OUT_X_L_A)?;
        Ok(I16x3 {
            x:(( u16(buffer[0]) + (u16(buffer[1]) << 8)) as i16 >> self.acc_shift) * self.acc_scale,
            y:(( u16(buffer[2]) + (u16(buffer[3]) << 8)) as i16 >> self.acc_shift) * self.acc_scale,
            z:(( u16(buffer[4]) + (u16(buffer[5]) << 8)) as i16 >> self.acc_shift) * self.acc_scale,
        })
    }
    
    /// Magnetometer measurements in milli Gauss
    pub fn mag(&mut self) -> Result<I16x3, E> {
        let buffer: GenericArray<u8, U6> = self.read_registers( MAG, OUTX_L_REG_M)?; 
        Ok(I16x3 {
            x: (u16(buffer[1]) + (u16(buffer[0]) << 8)) as i16 * 3 >> 1, // raw * 1.5
            y: (u16(buffer[3]) + (u16(buffer[2]) << 8)) as i16 * 3 >> 1, // raw * 1.5
            z: (u16(buffer[5]) + (u16(buffer[4]) << 8)) as i16 * 3 >> 1, // raw * 1.5
        })
    }

    /// Sets the magnetometer output data rate
    // pub fn mag_odr(&mut self, odr: MagOdr) -> Result<(), E> {
    //     self.modify_register(MAG, CRA_REG_M, |r| {
    //         r & !(0b111 << 2) | ((odr as u8) << 2)
    //     })
    // }

    /// Temperature sensor measurement
    ///
    /// - Resolution: 12-bit
    /// - Range: [-40, +85]
    pub fn temp(&mut self) -> Result<i16, E> {
        let buffer: GenericArray<u8, U2> = self.read_registers( ACCEL, OUT_TEMP_L_A)?;
        Ok(((u16(buffer[1]) + (u16(buffer[0]) << 8)) as i16) >> 4)
    }

    
    fn modify_register<F>(&mut self, sad: SlaveAddressEnum, reg: Register, f: F) -> Result<(), E>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self.read_register(sad, reg)?;
        self.write_register(sad, reg, f(r))?;
        Ok(())
    }

    fn read_registers<N>(&mut self, sad: SlaveAddressEnum, reg: Register)
                         -> Result<GenericArray<u8, N>, E>
    where
        N: ArrayLength<u8>,
    {
        let mut buffer: GenericArray<u8, N> = unsafe { mem::uninitialized() };
        
        {
            let buffer: &mut [u8] = &mut buffer;

            const MULTI: u8 = 1 << 7; // auto register increment for multiple reads
            self.i2c
                .write_read( sad as u8, &[reg as u8 | MULTI], buffer)?;
        }
        
        Ok(buffer)
    }
    
    fn read_register(&mut self, sad: SlaveAddressEnum, reg: Register) -> Result<u8, E> {
        self.read_registers::<U1>( sad, reg).map(|b| b[0])
    }

    fn write_register(&mut self, sad: SlaveAddressEnum, reg: Register, byte: u8) -> Result<(), E> {
        self.i2c.write( sad as u8, &[reg as u8, byte])
    }
       
    pub fn write<R>(&mut self, r: &R) -> Result<(),E> where
        R: RegBytes, 
    {
        self.i2c.write( R::SAD, r.as_ref())
    }

    pub fn read<R>(&mut self) -> Result<R, E> where
        R: RegBytes, 
    {
        let mut r = R::default();
        let (reg,b) = r.as_mut().split_at_mut(1);
        self.i2c.write_read( R::SAD, reg, b)?;
        Ok(r)
    }
}

/// XYZ triple
#[derive(Debug)]
pub struct I16x3 {
    /// X component
    pub x: i16,
    /// Y component
    pub y: i16,
    /// Z component
    pub z: i16,
}

/// Accellow power output data rate 
#[repr(u8)]
#[derive(Clone,Copy)]
#[allow(non_camel_case_types)]
pub enum Acc_Odr {
    Hz1     = 0b0001, Hz10    = 0b0010, Hz25    = 0b0011,
    Hz50    = 0b0100, Hz100   = 0b0101, Hz200   = 0b0110,
    Hz400   = 0b0111,
    LP_K1Hz620 = 0b1000,
    K1Hz344_LP_K5Hz376 = 0b1001,
    Undefined = 0b1111,
}
/// normal and high resolution data rates
#[repr(u8)]
#[derive(Clone,Copy)]
#[allow(non_camel_case_types)]
pub enum NHR_Odr {
    Hz1     = 0b0001, Hz10    = 0b0010, Hz25    = 0b0011,
    Hz50    = 0b0100, Hz100   = 0b0101, Hz200   = 0b0110,
    Hz400   = 0b0111, K1Hz344 = 0b1001
}

/// Acceleration sensitivity +-N g
#[derive(Clone,Copy)]
#[repr(u8)]
pub enum Sens {
    /// Range: [-2g, +2g]. Sensitivity ~ 1 g / (1 << 14) LSB
    G2 = 0b00,
    /// Range: [-4g, +4g]. Sensitivity ~ 2 g / (1 << 14) LSB
    G4 = 0b01,
    /// Range: [-8g, +8g]. Sensitivity ~ 4 g / (1 << 14) LSB
    G8 = 0b10,
    /// Range: [-16g, +16g]. Sensitivity ~ 12 g / (1 << 14) LSB
    G16 = 0x11,
}

impl Sens {
    pub fn scale( &self) -> i16 {
        1
        //match self { G2 => 1, G4 => 2, G8 => 4, G16 => 12 }
    }
}

#[derive(Clone,Copy)]
pub enum AccelMode {
    PowerDown,
    LowPower( NHR_Odr, Sens),
    Normal( NHR_Odr, Sens), 
    HighResolution( NHR_Odr, Sens),
}

/// Magnetometer Output Data Rate
#[derive(From,PartialEq,Debug)]
#[repr(u8)]
pub enum MagOdr {
    Hz10 = 0b00, Hz20 = 0b01,
    Hz50 = 0b10, Hz100 = 0b11
}

/// Magnetometer Operation Mode
#[derive(From,PartialEq,Debug)]
#[repr(u8)]
pub enum MagMode {
    Continuous = 0b00, SingleMeasurement = 0b01,
    Idle = 0b10, Idle_ = 0b11
}

/// Magnetometer Set Pulse
#[derive(From,PartialEq,Debug)]
#[repr(u8)]
pub enum MagSetPulseFreq {
    ReleaseEvery63ODR = 0b0,
    ReleaseAtPowerOnAfterPowerDown = 0b1
}


reg_bytes!{
    /// Magnetometer Configuration
    MagCfg, 4, CFG_REG_A_M, MAG; 
    /// magnetometer mode 
    MagMode, mode: rw 1,0,1;
    /// magnetometer output data rate
    MagOdr, odr: rw 1,2,3;
    /// low power mode enable, true is enabled,
    /// default is off  
    low_power: rw 1,4;
    /// Reset configuration and user registers,
    /// Flash registers keep their values.
    soft_reset: w 1,5;
    /// Reboot magnetometer memory content.
    reboot: w 1,6;
    /// Enable the magnetometer temperature
    /// compensation. Default value: false
    temp_comp: rw 1,7;
    /// Low-pass filter enable
    /// false -> bandwidth = odr/2,
    /// true -> bandwidth = odr/4
    low_pass_filter: rw 2,0;
    offset_cancelation: rw 2,1;
    /// Select frequency of the set pulse.
    MagSetPulseFreq, set_pulse_freq: rw 2,2;
    /// If true, the interrupt block recognition
    /// checks data after the hard-iron correction
    /// to discover the interrrupt
    int_on_data_off: rw 2,3;
    /// Enables offset cancellation in single
    /// measurement mode. The offset_cancelation
    /// must be true for enabling offset cancellation
    /// in single measurement mode.
    single_measurement_offset_cancelation: rw 2,4;
    /// If true, the DRDY pin is configured as a
    /// digital output.
    int_mag: rw 3,0;
    /// Enable self test
    self_test: rw 3,1;
    /// If true, an inversion of the low and high
    /// parts of the data occurs.
    ble: rw 3,3;
    /// If enabled, reading of incorrect data is
    /// avoided when the user reads asynchronously.
    /// In fact if the read request arrives during
    /// an update of the output data, a latch is
    /// possible, reading incoherent high and low
    /// parts of the same register. Only one part
    /// is updated and the other one remains old.
    bdu: rw 3,4;
    /// If true, the I 2 C interface is inhibited.
    /// Only the SPI interface can be used.
    i2c_disable: rw 3,5;
    /// If true, the INTERRUPT signal
    /// (INT bit inside INT_SOURCE_REG_M (64h))
    /// is driven on INT_MAG_PIN
    int_mag_pin: rw 3,6;    
}

#[repr(u8)]
#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Clone,Copy)]
pub enum SlaveAddressEnum {
    ACCEL = 0b0011001,
    MAG   = 0b0011110,
}

#[repr(u8)]
#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
enum Register {
    STATUS_REG_AUX_A = 0x07,
    OUT_TEMP_L_A = 0x0C,
    OUT_TEMP_H_A = 0x0D,
    INT_COUNTER_REG_A = 0x0E,
    WHO_AM_I_A = 0x0F,
    TEMP_CFG_REG_A = 0x1F,
    CTRL_REG1_A = 0x20,
    CTRL_REG2_A = 0x21,
    CTRL_REG3_A = 0x22,
    CTRL_REG4_A = 0x23,
    CTRL_REG5_A = 0x24,
    CTRL_REG6_A = 0x25,
    REFERENCE_A = 0x26,
    STATUS_REG_A = 0x27,
    OUT_X_L_A = 0x28,
    OUT_X_H_A = 0x29,
    OUT_Y_L_A = 0x2A,
    OUT_Y_H_A = 0x2B,
    OUT_Z_L_A = 0x2C,
    OUT_Z_H_A = 0x2D,
    FIFO_CTRL_REG_A = 0x2E,
    FIFO_SRC_REG_A = 0x2F,
    INT1_CFG_A = 0x30,
    INT1_SRC_A = 0x31,
    INT1_THS_A = 0x32,
    INT1_DURATION_A = 0x33,
    INT2_CFG_A = 0x34,
    INT2_SRC_A = 0x35,
    INT2_THS_A = 0x36,
    INT2_DURATION_A = 0x37,
    CLICK_CFG_A = 0x38,
    CLICK_SRC_A = 0x39,
    CLICK_THS_A = 0x3A,
    TIME_LIMIT_A = 0x3B,
    TIME_LATENCY_A = 0x3C,
    TIME_WINDOW_A = 0x3D,
    Act_THS_A = 0x3E,
    Act_DUR_A = 0x3F,
    OFFSET_X_REG_L_M = 0x45,
    OFFSET_X_REG_H_M = 0x46,
    OFFSET_Y_REG_L_M = 0x47,
    OFFSET_Y_REG_H_M = 0x48,
    OFFSET_Z_REG_L_M = 0x49,
    OFFSET_Z_REG_H_M = 0x4A,
    WHO_AM_I_M = 0x4F,
    CFG_REG_A_M = 0x60,
    CFG_REG_B_M = 0x61,
    CFG_REG_C_M = 0x62,
    INT_CTRL_REG_M = 0x63,
    INT_SOURCE_REG_M = 0x64,
    INT_THS_L_REG_M = 0x65,
    INT_THS_H_REG_M = 0x66,
    STATUS_REG_M = 0x67,
    OUTX_L_REG_M = 0x68,
    OUTX_H_REG_M = 0x69,
    OUTY_L_REG_M = 0x6A,
    OUTY_H_REG_M = 0x6B,
    OUTZ_L_REG_M = 0x6C,
    OUTZ_H_REG_M = 0x6D,
}

