#![no_std]

//! Manages a new EM7180 sensor hub with MPU9250 gyro/accelerometer,
//! embedded Asahi Kasei AK8963C magnetometer, Measurement Specialties'
//! MS5637 Barometer and ST's M24512DFC I2C EEPROM module

#![deny(
    missing_docs,
    missing_debug_implementations,
    missing_copy_implementations,
    trivial_casts,
    unstable_features,
    unused_import_braces,
    unused_qualifications,
    //warnings
)]
#![allow(dead_code, clippy::uninit_assumed_init, clippy::too_many_arguments)]

extern crate cast;
extern crate embedded_hal as ehal;
extern crate generic_array;
extern crate nb;
extern crate safe_transmute;

use ::core::mem::MaybeUninit;

use ehal::blocking::i2c::WriteRead;
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};

/// Sometimes it's correct (0x28 << 1) instead of 0x28
const EM7180_ADDRESS: u8 = 0x28;

/// Struct for USFS
#[derive(Debug, Copy, Clone)]
pub struct USFS<I2C> {
    com: I2C,
    address: u8,
    int_pin: u8,
    pass_thru: bool,
}

/// Defines errors
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// WHO_AM_I returned invalid value (returned value is argument)
    InvalidDevice(u8),
    /// Underlying bus error
    BusError(E),
    /// Timeout
    Timeout,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}

impl<I2C, E> USFS<I2C>
where
    I2C: WriteRead<Error = E>,
{
    /// Creates a sensor with default configuration
    pub fn default(i2c: I2C) -> Result<USFS<I2C>, Error<E>>
    where
        I2C: WriteRead<Error = E>,
    {
        USFS::new(i2c, EM7180_ADDRESS, 8, false)
    }

    /// Creates a sensor with specific configuration
    pub fn new(i2c: I2C, address: u8, int_pin: u8, pass_thru: bool) -> Result<USFS<I2C>, Error<E>>
    where
        I2C: WriteRead<Error = E>,
    {
        let mut chip = USFS {
            com: i2c,
            address,
            int_pin,
            pass_thru,
        };

        let wai = chip.get_product_id()?;

        if wai == 0x80 {
            /*
                Choose EM7180, MPU9250 and MS5637 sample rates and bandwidths
                Choices are:
                accBW, gyroBW 0x00 = 250 Hz, 0x01 = 184 Hz, 0x02 = 92 Hz, 0x03 = 41 Hz, 0x04 = 20 Hz, 0x05 = 10 Hz, 0x06 = 5 Hz, 0x07 = no filter (3600 Hz)
                QRtDiv 0x00, 0x01, 0x02, etc quat rate = gyroRt/(1 + QRtDiv)
                magRt 8 Hz = 0x08 or 100 Hz 0x64
                accRt, gyroRt 1000, 500, 250, 200, 125, 100, 50 Hz enter by choosing desired rate
                and dividing by 10, so 200 Hz would be 200/10 = 20 = 0x14
                sample rate of barometer is baroRt/2 so for 25 Hz enter 50 = 0x32

                uint8_t accBW = 0x03, gyroBW = 0x03, QRtDiv = 0x01, magRt = 0x64, accRt = 0x14, gyroRt = 0x14, baroRt = 0x32;

                Choose MPU9250 sensor full ranges
                Choices are 2, 4, 8, 16 g for accFS, 250, 500, 1000, and 2000 dps for gyro FS and 1000 uT for magFS expressed as HEX values

                uint16_t accFS = 0x08, gyroFS = 0x7D0, magFS = 0x3E8;
            */

            let acc_bw: u8 = 0x03;
            let gyro_bw: u8 = 0x03;
            let qrt_div: u8 = 0x01;
            let mag_rt: u8 = 0x64;
            let acc_rt: u8 = 0x14;
            let gyro_rt: u8 = 0x14;
            let baro_rt: u8 = 0x32;
            let acc_fs: u16 = 0x08;
            let gyro_fs: u16 = 0x7D0;
            let mag_fs: u16 = 0x3E8;

            chip.load_fw_from_eeprom()?;
            chip.init_hardware(
                acc_bw, gyro_bw, acc_fs, gyro_fs, mag_fs, qrt_div, mag_rt, acc_rt, gyro_rt, baro_rt,
            )?;
            Ok(chip)
        } else {
            Err(Error::InvalidDevice(wai))
        }
    }

    fn read_byte(&mut self, reg: u8) -> Result<u8, E> {
        let mut data: [u8; 1] = [0];
        self.com.write_read(self.address, &[reg], &mut data)?;
        Ok(data[0])
    }

    fn read_registers<N>(&mut self, reg: Register) -> Result<GenericArray<u8, N>, E>
    where
        N: ArrayLength<u8>,
    {
        let mut buffer: GenericArray<u8, N> =
            unsafe { MaybeUninit::<GenericArray<u8, N>>::uninit().assume_init() };

        {
            let buffer: &mut [u8] = &mut buffer;
            const I2C_AUTO_INCREMENT: u8 = 0;
            self.com
                .write_read(self.address, &[(reg as u8) | I2C_AUTO_INCREMENT], buffer)?;
        }

        Ok(buffer)
    }

    fn read_4bytes(&mut self, reg: Register) -> Result<[u8; 4], E> {
        let buffer: GenericArray<u8, U4> = self.read_registers(reg)?;
        let mut ret: [u8; 4] = Default::default();
        ret.copy_from_slice(buffer.as_slice());

        Ok(ret)
    }

    fn read_6bytes(&mut self, reg: Register) -> Result<[u8; 6], E> {
        let buffer: GenericArray<u8, U6> = self.read_registers(reg)?;
        let mut ret: [u8; 6] = Default::default();
        ret.copy_from_slice(buffer.as_slice());

        Ok(ret)
    }

    fn read_16bytes(&mut self, reg: Register) -> Result<[u8; 16], E> {
        let buffer: GenericArray<u8, U16> = self.read_registers(reg)?;
        let mut ret: [u8; 16] = Default::default();
        ret.copy_from_slice(buffer.as_slice());

        Ok(ret)
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, E> {
        let mut data: [u8; 1] = [0];
        self.com.write_read(self.address, &[reg as u8], &mut data)?;
        Ok(data[0])
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), E> {
        let mut buffer = [0];
        self.com
            .write_read(self.address, &[reg as u8, byte], &mut buffer)
    }

    fn em7180_set_integer_param(&mut self, param: u8, param_val: u32) -> Result<(), E> {
        let bytes_0 = (param_val & (0xFF)) as u8;
        let bytes_1 = ((param_val >> 8) & (0xFF)) as u8;
        let bytes_2 = ((param_val >> 16) & (0xFF)) as u8;
        let bytes_3 = ((param_val >> 24) & (0xFF)) as u8;
        let param = param | 0x80; // Parameter is the decimal value with the MSB set high to indicate a paramter write processs

        self.write_register(Register::EM7180_LoadParamByte0, bytes_0)?; // Param LSB
        self.write_register(Register::EM7180_LoadParamByte1, bytes_1)?;
        self.write_register(Register::EM7180_LoadParamByte2, bytes_2)?;
        self.write_register(Register::EM7180_LoadParamByte3, bytes_3)?; // Param MSB
        self.write_register(Register::EM7180_ParamRequest, param)?;
        self.write_register(Register::EM7180_AlgorithmControl, 0x80)?; // Request parameter transfer procedure
        let mut stat = self.read_register(Register::EM7180_ParamAcknowledge)?; // Check the parameter acknowledge register and loop until the result matches parameter request byte

        while stat != param {
            stat = self.read_register(Register::EM7180_ParamAcknowledge)?;
        }

        self.write_register(Register::EM7180_ParamRequest, 0x00)?; // Parameter request = 0 to end parameter transfer process
        self.write_register(Register::EM7180_AlgorithmControl, 0x00)?; // Re-start algorithm

        Ok(())
    }

    fn em7180_set_mag_acc_fs(&mut self, mag_fs: u16, acc_fs: u16) -> Result<(), E> {
        let bytes_0 = (mag_fs & (0xFF)) as u8;
        let bytes_1 = ((mag_fs >> 8) & (0xFF)) as u8;
        let bytes_2 = (acc_fs & (0xFF)) as u8;
        let bytes_3 = ((acc_fs >> 8) & (0xFF)) as u8;

        self.write_register(Register::EM7180_LoadParamByte0, bytes_0)?; // Mag LSB
        self.write_register(Register::EM7180_LoadParamByte1, bytes_1)?; // Mag MSB
        self.write_register(Register::EM7180_LoadParamByte2, bytes_2)?; // Acc LSB
        self.write_register(Register::EM7180_LoadParamByte3, bytes_3)?; // Acc MSB
        self.write_register(Register::EM7180_ParamRequest, 0xCA)?; // Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
        self.write_register(Register::EM7180_AlgorithmControl, 0x80)?; // Request parameter transfer procedure
        let mut stat = self.read_register(Register::EM7180_ParamAcknowledge)?; // Check the parameter acknowledge register and loop until the result matches parameter request byte

        while stat != 0xCA {
            stat = self.read_register(Register::EM7180_ParamAcknowledge)?;
        }

        self.write_register(Register::EM7180_ParamRequest, 0x00)?; // Parameter request = 0 to end parameter transfer process
        self.write_register(Register::EM7180_AlgorithmControl, 0x00)?; // Re-start algorithm

        Ok(())
    }

    fn em7180_set_gyro_fs(&mut self, gyro_fs: u16) -> Result<(), E> {
        let bytes_0 = (gyro_fs & (0xFF)) as u8;
        let bytes_1 = ((gyro_fs >> 8) & (0xFF)) as u8;
        let bytes_2 = 0x00;
        let bytes_3 = 0x00;

        self.write_register(Register::EM7180_LoadParamByte0, bytes_0)?; // Gyro LSB
        self.write_register(Register::EM7180_LoadParamByte1, bytes_1)?; // Gyro MSB
        self.write_register(Register::EM7180_LoadParamByte2, bytes_2)?; // Unused
        self.write_register(Register::EM7180_LoadParamByte3, bytes_3)?; // Unused
        self.write_register(Register::EM7180_ParamRequest, 0xCB)?; // Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
        self.write_register(Register::EM7180_AlgorithmControl, 0x80)?; // Request parameter transfer procedure
        let mut stat = self.read_register(Register::EM7180_ParamAcknowledge)?; // Check the parameter acknowledge register and loop until the result matches parameter request byte

        while stat != 0xCB {
            stat = self.read_register(Register::EM7180_ParamAcknowledge)?;
        }

        self.write_register(Register::EM7180_ParamRequest, 0x00)?; // Parameter request = 0 to end parameter transfer process
        self.write_register(Register::EM7180_AlgorithmControl, 0x00)?; // Re-start algorithm

        Ok(())
    }

    fn load_fw_from_eeprom(&mut self) -> Result<(), E> {
        let mut stat = self.read_register(Register::EM7180_SentralStatus)? & 0x01;

        let mut count = 0;
        while stat != 0 {
            self.write_register(Register::EM7180_ResetRequest, 0x01)?;
            count += 1;
            stat = self.read_register(Register::EM7180_SentralStatus)? & 0x01;
            if count > 10 {
                break;
            };
        }

        Ok(())
    }

    fn init_hardware(
        &mut self,
        acc_bw: u8,
        gyro_bw: u8,
        acc_fs: u16,
        gyro_fs: u16,
        mag_fs: u16,
        qrt_div: u8,
        mag_rt: u8,
        acc_rt: u8,
        gyro_rt: u8,
        baro_rt: u8,
    ) -> Result<(), Error<E>> {
        self.write_register(Register::EM7180_HostControl, 0x00)?; // Set SENtral in initialized state to configure registers
        self.write_register(Register::EM7180_PassThruControl, 0x00)?; // Make sure pass through mode is off
        self.write_register(Register::EM7180_HostControl, 0x01)?; // Force initialize
        self.write_register(Register::EM7180_HostControl, 0x00)?; // Set SENtral in initialized state to configure registers

        // Setup LPF bandwidth (BEFORE setting ODR's)
        self.write_register(Register::EM7180_ACC_LPF_BW, acc_bw)?; // accBW = 3 = 41Hz
        self.write_register(Register::EM7180_GYRO_LPF_BW, gyro_bw)?; // gyroBW = 3 = 41Hz
                                                                     // Set accel/gyro/mag desired ODR rates
        self.write_register(Register::EM7180_QRateDivisor, qrt_div)?; // quat rate = gyroRt/(1 QRTDiv)
        self.write_register(Register::EM7180_MagRate, mag_rt)?; // 0x64 = 100 Hz
        self.write_register(Register::EM7180_AccelRate, acc_rt)?; // 200/10 Hz, 0x14 = 200 Hz
        self.write_register(Register::EM7180_GyroRate, gyro_rt)?; // 200/10 Hz, 0x14 = 200 Hz
        self.write_register(Register::EM7180_BaroRate, 0x80 | baro_rt)?; // Set enable bit and set Baro rate to 25 Hz, rate = baroRt/2, 0x32 = 25 Hz

        // Configure operating mode
        self.write_register(Register::EM7180_AlgorithmControl, 0x00)?; // Read scale sensor data
                                                                       // Enable interrupt to host upon certain events
                                                                       // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
                                                                       // New mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
        self.write_register(Register::EM7180_EnableEvents, 0x07)?;
        // Enable EM7180 run mode
        self.write_register(Register::EM7180_HostControl, 0x01)?; // Set SENtral in normal run mode

        // delay(100);

        // Read sensor default FS values from parameter space
        self.write_register(Register::EM7180_ParamRequest, 0x4A)?; // Request to read parameter 74

        self.write_register(Register::EM7180_AlgorithmControl, 0x80)?; // Request parameter transfer process
        let mut param_xfer = self.read_register(Register::EM7180_ParamAcknowledge)?;
        while param_xfer != 0x4A {
            param_xfer = self.read_register(Register::EM7180_ParamAcknowledge)?;
        }

        self.write_register(Register::EM7180_ParamRequest, 0x4B)?; // Request to read  parameter 75
        let mut param_xfer = self.read_register(Register::EM7180_ParamAcknowledge)?;
        while param_xfer != 0x4B {
            param_xfer = self.read_register(Register::EM7180_ParamAcknowledge)?;
        }

        self.write_register(Register::EM7180_ParamRequest, 0x00)?; //End parameter transfer
        self.write_register(Register::EM7180_AlgorithmControl, 0x00)?; // re-enable algorithm

        // Disable stillness mode for balancing robot application
        self.em7180_set_integer_param(0x49, 0x00)?;

        // Write desired sensor full scale ranges to the EM7180
        self.em7180_set_mag_acc_fs(mag_fs, acc_fs)?; // 1000 uT == 0x3E8, 8 g == 0x08
        self.em7180_set_gyro_fs(gyro_fs)?; // 2000 dps == 0x7D0

        // Read sensor new FS values from parameter space
        self.write_register(Register::EM7180_ParamRequest, 0x4A)?; // Request to read  parameter 74
        self.write_register(Register::EM7180_AlgorithmControl, 0x80)?; // Request parameter transfer process
        let mut param_xfer = self.read_register(Register::EM7180_ParamAcknowledge)?;
        while param_xfer != 0x4A {
            param_xfer = self.read_register(Register::EM7180_ParamAcknowledge)?;
        }

        self.write_register(Register::EM7180_ParamRequest, 0x4B)?; // Request to read  parameter 75
        let mut param_xfer = self.read_register(Register::EM7180_ParamAcknowledge)?;
        while param_xfer != 0x4B {
            param_xfer = self.read_register(Register::EM7180_ParamAcknowledge)?;
        }

        self.write_register(Register::EM7180_ParamRequest, 0x00)?; //End parameter transfer
        self.write_register(Register::EM7180_AlgorithmControl, 0x00)?; // re-enable algorithm

        Ok(())
    }

    /// Get ProductID, should be: 0x80
    pub fn get_product_id(&mut self) -> Result<u8, E> {
        self.read_register(Register::EM7180_ProductID)
    }

    /// Get ProductID, should be: 0xE609
    pub fn get_rom_version(&mut self) -> Result<[u8; 2], E> {
        let rom_version1 = self.read_register(Register::EM7180_ROMVersion1)?;
        let rom_version2 = self.read_register(Register::EM7180_ROMVersion2)?;
        Ok([rom_version1, rom_version2])
    }

    /// Get Sentral status
    pub fn get_sentral_status(&mut self) -> Result<u8, E> {
        self.read_register(Register::EM7180_SentralStatus)
    }

    /// Get run status
    pub fn check_run_status(&mut self) -> Result<u8, E> {
        self.read_register(Register::EM7180_RunStatus)
    }

    /// Check EventStatus on address 0x35
    /// Non-zero value indicates a new event has been generated
    /// bit | meaning
    ///  0  | CPUReset
    ///  1  | Error
    ///  2  | QuaternionResult
    ///  3  | MagResult
    ///  4  | AccelResult
    ///  5  | GyroResult
    pub fn check_status(&mut self) -> Result<u8, E> {
        self.read_register(Register::EM7180_EventStatus)
    }

    /// Check ErrorRegister Software-Related Error Conditions on address 0x50
    /// Non-zero value indicates an error
    /// value | error condition                         | response
    /// 0x00  | no error                                |
    /// 0x80  | invalid sample rate selected            | check sensor rate settings
    /// 0x30  | mathematical error                      | check for software updates
    /// 0x21  | Magnetometer initialization failed      | this error can be caused by a wrong driver,
    /// 0x22  | accelerometer initialization failed     | physically bad sensor connection, or
    /// 0x24  | gyroscope initialization failed         | incorrect I2C device address in the driver
    /// 0x11  | magnetometer rate failure               | this error indicates the given sensor
    /// 0x12  | accelerometer rate failure              | is unreliable and has stopped
    /// 0x14  | Gyroscope rate failure                  | producing data
    pub fn check_errors(&mut self) -> Result<u8, E> {
        self.read_register(Register::EM7180_Error)
    }

    /// Check SensorStatus Sensor-Related Error Conditions on address 0x36
    /// Non-zero value indicates sensor-related error
    /// bit | meaning
    ///  0  | MagNACK. 1 = NACK from magnetometer
    ///  1  | AccelNACK. 1 = NACK from accelerometer
    ///  2  | GyroNACK. 1 = NACK from gyroscope
    ///  4  | MagDeviceIDErr. 1 = Unexpected DeviceID from magnetometer
    ///  5  | AccelDeviceIDErr. 1 = Unexpected DeviceID from accelerometer
    ///  6  | GyroDeviceIDErr. 1 = Unexpected DeviceID from gyroscope
    pub fn check_sensor_status(&mut self) -> Result<u8, E> {
        self.read_register(Register::EM7180_SensorStatus)
    }

    /// Check which sensors can be detected by the EM7180
    /// value | response
    /// 0x01  | A barometer is installed
    /// 0x02  | A humidity sensor is installed
    /// 0x04  | A temperature sensor is installed
    /// 0x08  | A custom sensor is installed
    /// 0x10  | A second custom sensor is installed
    /// 0x20  | A third custom sensor is installed
    pub fn check_feature_flags(&mut self) -> Result<u8, E> {
        self.read_register(Register::EM7180_FeatureFlags)
    }

    /// Get actual magnetometer output data rate
    pub fn get_actual_magnetometer_rate(&mut self) -> Result<u8, E> {
        Ok(self.read_register(Register::EM7180_ActualMagRate)?)
    }

    /// Get actual accelerometer output data rate
    pub fn get_actual_accel_rate(&mut self) -> Result<u8, E> {
        Ok(self.read_register(Register::EM7180_ActualAccelRate)? * 10)
    }

    /// Get actual gyroscope output data rate
    pub fn get_actual_gyroscope_rate(&mut self) -> Result<u8, E> {
        Ok(self.read_register(Register::EM7180_ActualGyroRate)? * 10)
    }

    /// Read Linear Acceleration
    /// AX Linear Acceleration – X Axis
    /// AY Linear Acceleration – Y Axis
    /// AZ Linear Acceleration – Z Axis
    pub fn read_sentral_accel_data(&mut self) -> Result<[i16; 3], E> {
        let raw_data = self.read_6bytes(Register::EM7180_AX)?;

        let ax = (raw_data[1] as i16) << 8 | raw_data[0] as i16;
        let ay = (raw_data[3] as i16) << 8 | raw_data[2] as i16;
        let az = (raw_data[5] as i16) << 8 | raw_data[4] as i16;

        Ok([ax, ay, az])
    }

    /// Read Rotational Velocity
    /// GX Rotational Velocity – X Axis
    /// GY Rotational Velocity – Y Axis
    /// GZ Rotational Velocity – Z Axis
    pub fn read_sentral_gyro_data(&mut self) -> Result<[i16; 3], E> {
        let raw_data = self.read_6bytes(Register::EM7180_GX)?;

        let ax = (raw_data[1] as i16) << 8 | raw_data[0] as i16;
        let ay = (raw_data[3] as i16) << 8 | raw_data[2] as i16;
        let az = (raw_data[5] as i16) << 8 | raw_data[4] as i16;

        Ok([ax, ay, az])
    }

    /// Read Magnetic Field
    /// MX Magnetic Field – X Axis
    /// MY Magnetic Field – Y Axis
    /// MZ Magnetic Field – Z Axis
    pub fn read_sentral_mag_data(&mut self) -> Result<[i16; 3], E> {
        let raw_data = self.read_6bytes(Register::EM7180_MX)?;

        let ax = (raw_data[1] as i16) << 8 | raw_data[0] as i16;
        let ay = (raw_data[3] as i16) << 8 | raw_data[2] as i16;
        let az = (raw_data[5] as i16) << 8 | raw_data[4] as i16;

        Ok([ax, ay, az])
    }

    /// Read Normalized Quaternion
    /// QX Normalized Quaternion – X, or Heading    | Full-Scale Range 0.0 – 1.0 or ±π
    /// QY Normalized Quaternion – Y, or Pitch      | Full-Scale Range 0.0 – 1.0 or ±π/2
    /// QZ Normalized Quaternion – Z, or Roll       | Full-Scale Range 0.0 – 1.0 or ±π
    /// QW Normalized Quaternion – W, or 0.0        | Full-Scale Range 0.0 – 1.0
    pub fn read_sentral_quat_qata(&mut self) -> Result<[f32; 4], E> {
        let raw_data_qx = self.read_4bytes(Register::EM7180_QX)?;
        let qx = reg_to_float(&raw_data_qx);

        let raw_data_qy = self.read_4bytes(Register::EM7180_QY)?;
        let qy = reg_to_float(&raw_data_qy);

        let raw_data_qz = self.read_4bytes(Register::EM7180_QZ)?;
        let qz = reg_to_float(&raw_data_qz);

        let raw_data_qw = self.read_4bytes(Register::EM7180_QW)?;
        let qw = reg_to_float(&raw_data_qw);

        Ok([qx, qy, qz, qw])
    }

    /// Read Pressure in mBar
    pub fn read_sentral_baro_data(&mut self) -> Result<i16, E> {
        let raw_data = self.read_6bytes(Register::EM7180_Baro)?; // Read the two raw data registers sequentially into data array
        Ok((raw_data[1] as i16) << 8 | raw_data[0] as i16) // Turn the MSB and LSB into a signed 16-bit value
    }

    /// Read Temperature in °C
    pub fn read_sentral_temp_data(&mut self) -> Result<i16, E> {
        let raw_data = self.read_6bytes(Register::EM7180_Temp)?; // Read the two raw data registers sequentially into data array
        Ok((raw_data[1] as i16) << 8 | raw_data[0] as i16) // Turn the MSB and LSB into a signed 16-bit value
    }
}

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
enum Register {
    EM7180_EventStatus = 0x35,
    EM7180_Error = 0x50,
    EM7180_ProductID = 0x90,
    EM7180_HostControl = 0x34,
    EM7180_PassThruControl = 0xA0,
    EM7180_ACC_LPF_BW = 0x5B,
    EM7180_GYRO_LPF_BW = 0x5C,
    EM7180_QRateDivisor = 0x32,
    EM7180_MagRate = 0x55,
    EM7180_AccelRate = 0x56,
    EM7180_GyroRate = 0x57,
    EM7180_BaroRate = 0x58,
    EM7180_AlgorithmControl = 0x54,
    EM7180_EnableEvents = 0x33,
    EM7180_SensorStatus = 0x36,
    EM7180_ActualMagRate = 0x45,
    EM7180_ActualAccelRate = 0x46,
    EM7180_ActualGyroRate = 0x47,
    EM7180_ParamRequest = 0x64,
    EM7180_ParamAcknowledge = 0x3A,
    EM7180_ROMVersion1 = 0x70,
    EM7180_ROMVersion2 = 0x71,
    EM7180_LoadParamByte0 = 0x60,
    EM7180_LoadParamByte1 = 0x61,
    EM7180_LoadParamByte2 = 0x62,
    EM7180_LoadParamByte3 = 0x63,
    EM7180_SavedParamByte0 = 0x3B,
    EM7180_SavedParamByte1 = 0x3C,
    EM7180_SavedParamByte2 = 0x3D,
    EM7180_SavedParamByte3 = 0x3E,
    EM7180_FeatureFlags = 0x39,
    EM7180_SentralStatus = 0x37,
    EM7180_ResetRequest = 0x9B,
    EM7180_RunStatus = 0x92,
    EM7180_QX = 0x00, // this is a 32-bit normalized floating point number read from registers 0x00-03
    EM7180_QY = 0x04, // this is a 32-bit normalized floating point number read from registers 0x04-07
    EM7180_QZ = 0x08, // this is a 32-bit normalized floating point number read from registers 0x08-0B
    EM7180_QW = 0x0C, // this is a 32-bit normalized floating point number read from registers 0x0C-0F
    EM7180_AX = 0x1A, // i16 from registers 0x1A-1B
    EM7180_GX = 0x22, // i16 from registers 0x22-23
    EM7180_MX = 0x12, // int16_t from registers 0x12-13
    EM7180_Baro = 0x2A, // start of two-byte MS5637 pressure data, 16-bit signed interger
    EM7180_Temp = 0x2E, // start of two-byte MS5637 temperature data, 16-bit signed interger
}

/// Transform raw quaternion buffer data into something meaningfull
fn reg_to_float(buf: &[u8]) -> f32 {
    unsafe { safe_transmute::guarded_transmute::<f32>(buf).unwrap() }
}

/// Transform raw temperature data into Celsius degrees
pub fn raw_temperature_to_celsius(raw_temperature: i16) -> f64 {
    raw_temperature as f64 * 0.01f64
}

/// Transform raw temperature data into Fahrenheit degrees
pub fn raw_temperature_to_fahrenheit(raw_temperature: i16) -> f64 {
    9.0f64 * raw_temperature_to_celsius(raw_temperature) / 5.0f64 + 32.0f64
}

/// Transform raw pressure data into mbar pressure
pub fn raw_pressure_to_mbar(raw_pressure: i16) -> f64 {
    raw_pressure as f64 * 0.01f64 + 1013.25f64
}

/// Transform raw pressure data into feet altitude
pub fn raw_pressure_to_feet(_raw_pressure: i16) -> f64 {
    // powf error, let's check
    //let x = raw_pressure_to_mbar(raw_pressure) / 1013.25f64;
    //145366.45f64 * (1.0f64 - x.powf(0.190284f64))
    0.0
}
