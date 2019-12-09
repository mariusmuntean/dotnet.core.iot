using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using Iot.Device.Lsm9Ds1;
using Unosquare.RaspberryIO;
using Unosquare.RaspberryIO.Abstractions;
using Unosquare.WiringPi;

namespace dotnet.core.iot
{
    public partial class LSM9DS1
    {
        const float gravitationalConstantEarth = 9.80665f;

        private const byte AccGyroAddress = 0x6B;
        private const byte MagAddress = 0x1E;

        // identities
        private const byte AccGyroIdentity = 0b01101000;
        private const byte MagIdentity = 0b00111101;

        private const byte MagAddress1 = 0x1E;
        private const byte MagAddress2 = 0x1C;
        private const byte AccGyroAddress1 = 0x6B;
        private const byte AccGyroAddress2 = 0x6A;

        // registers
        private const byte WHO_AM_I_AccGyro = 0x0f;
        private const byte WHO_AM_I_Mag = 0x0f;

        private const byte CTRL_REG1_G = 0b0001_0000;
        private const byte CTRL_REG2_G = 0b0001_0001;
        private const byte CTRL_REG3_G = 0b0001_0010;

        private const byte CTRL_REG1_M = 0x20;
        private const byte CTRL_REG2_M = 0x21;
        private const byte CTRL_REG3_M = 0x22;
        private const byte CTRL_REG4_M = 0x23;
        private const byte CTRL_REG5_M = 0x24;

        private const byte CTRL_REG5_XL = 0x1F;

        /// <summary>
        /// Linear acceleration sensor Control Register 6
        /// </summary>
        private const byte CTRL_REG6_XL = 0x20;

        private const byte CTRL_REG7_XL = 0x21;

        private const byte STATUS_REG = 0b00010111;

        private const byte OUT_X_L_G = 0x18;
        private const byte OUT_X_H_G = 0x19;
        private const byte OUT_Y_L_G = 0x1A;
        private const byte OUT_Y_H_G = 0x1B;
        private const byte OUT_Z_L_G = 0x1C;
        private const byte OUT_Z_H_G = 0x1D;

        private const byte OUT_X_L_XL = 0x28;
        private const byte OUT_X_H_XL = 0x29;
        private const byte OUT_Y_L_XL = 0x2A;
        private const byte OUT_Y_H_XL = 0x2B;
        private const byte OUT_Z_L_XL = 0x2C;
        private const byte OUT_Z_H_XL = 0x2D;

        private const byte OUT_X_L_M = 0x28;
        private const byte OUT_X_H_M = 0x29;
        private const byte OUT_Y_L_M = 0x2A;
        private const byte OUT_Y_H_M = 0x2B;
        private const byte OUT_Z_L_M = 0x2C;
        private const byte OUT_Z_H_M = 0x2D;

        private const byte FIFO_CTRL = 0x2E;
        private const byte CTRL_REG4 = 0x1E;
        private const byte CTRL_REG8 = 0x22;

        static II2CDevice _accGyroDevice;
        static II2CDevice _magDevice;


        private Dictionary<LinearAccelerationScale, float> _accelerationToConversionFactor = new Dictionary<LinearAccelerationScale, float>
        {
            {LinearAccelerationScale._2G, 0.061f},
            {LinearAccelerationScale._4G, 0.122f},
            {LinearAccelerationScale._8G, 0.244f},
            {LinearAccelerationScale._16G, 0.732f}
        };

        public LSM9DS1()
        {
            Pi.Init<BootstrapWiringPi>();

            Console.WriteLine(Pi.Info.ToString());

            _accGyroDevice = Pi.I2C.AddDevice(AccGyroAddress);
            _magDevice = Pi.I2C.AddDevice(MagAddress);
        }

        public void Begin()
        {
            // Soft reset and reboot acc/gyro
            SoftResetRebootAccGyro();

            // Soft reset and reboot magnetometer
            SoftResetRebootMagnetometer();

            // Wait for reboot
            Thread.Sleep(10);

            CheckIdentity();

            // Enable gyro continuous
            SetGyroContinuous();

            // Enable accelerometer continuous
            SetAccContinuous();

            // Enable magnetometer continuous
            SetMagContinuous();

            // Set default ranges for the 3 sensors
            SetAccScale(LinearAccelerationScale._16G);
            SetGyroScale(AngularRateScale._2000);
            SetMagnetometerRange(MagneticSensitivityScale._4Gauss);
        }

        public Vector3 ReadAccelerometer()
        {
            var rawValues = GetAccRaw();

            var scaleFactor = _accelerationToConversionFactor[AccelerometerScale];
            scaleFactor /= 1000.0f;

            return new Vector3(
                rawValues.X *= scaleFactor * gravitationalConstantEarth,
                rawValues.Y *= scaleFactor * gravitationalConstantEarth,
                rawValues.Z *= scaleFactor * gravitationalConstantEarth
            );
        }

        public Vector3 ReadMagnetometer()
        {
            var rawValues = GetMagRaw();

            var scaleFactor = _magneticSensitivityToScalingFactor[MagnetometerScale];
            scaleFactor /= 1000.0f;

            return new Vector3(
                rawValues.X *= scaleFactor,
                rawValues.Y *= scaleFactor,
                rawValues.Z *= scaleFactor
            );
        }

        public Vector3 ReadGyroscope()
        {
            var rawValues = GetGyroRaw();

            var scaleFactor = _angularRateToScalingFactor[GyroScale];

            return new Vector3(
                rawValues.X *= scaleFactor,
                rawValues.Y *= scaleFactor,
                rawValues.Z *= scaleFactor
            );
        }

        private Dictionary<AngularRateScale, float> _angularRateToScalingFactor = new Dictionary<AngularRateScale, float>()
        {
            {AngularRateScale._245, 0.00875f},
            {AngularRateScale._500, 0.0175f},
            {AngularRateScale._2000, 0.0700f}
        };

        private Dictionary<MagneticSensitivityScale, float> _magneticSensitivityToScalingFactor = new Dictionary<MagneticSensitivityScale, float>()
        {
            {MagneticSensitivityScale._4Gauss, 0.14f},
            {MagneticSensitivityScale._8Gauss, 0.29f},
            {MagneticSensitivityScale._12Gauss, 0.43f},
            {MagneticSensitivityScale._16Gauss, 0.58f}
        };

        private void SetMagnetometerRange(MagneticSensitivityScale sensitivityScale)
        {
            // get current register data
            var registerData = _accGyroDevice.ReadAddressByte(CTRL_REG2_M);

            // zero out scale bits
            registerData &= BitConverter.GetBytes(~ 0b0_11_00000)[0];

            // set new scale
            registerData |= (byte) sensitivityScale;
            _accGyroDevice.WriteAddressByte(CTRL_REG2_M, registerData);

            MagnetometerScale = sensitivityScale;
        }

        public MagneticSensitivityScale MagnetometerScale { get; private set; }

        private void SetGyroScale(AngularRateScale angularRateScale)
        {
            // get current register data
            var registerData = _accGyroDevice.ReadAddressByte(CTRL_REG1_G);

            // zero out scale bits
            registerData &= BitConverter.GetBytes(~0b000_11_000)[0];

            // set new scale
            registerData |= (byte) angularRateScale;
            _accGyroDevice.WriteAddressByte(CTRL_REG1_G, registerData);

            GyroScale = angularRateScale;
        }

        public AngularRateScale GyroScale { get; set; }

        private void SetAccScale(LinearAccelerationScale linearAccelerationScale)
        {
            // Get current register data
            var registerData = _accGyroDevice.ReadAddressByte(CTRL_REG6_XL);

            // Zero out the range bits
            registerData &= BitConverter.GetBytes(~ 0b0001_1000)[0]; // ToByte() throwsOverflowException is the int is larger than max value of Byte. This should not be the case here.

            // Set to desired values
            registerData |= (byte) linearAccelerationScale;
            _accGyroDevice.WriteAddressByte(CTRL_REG6_XL, registerData);

            AccelerometerScale = linearAccelerationScale;
        }

        public LinearAccelerationScale AccelerometerScale { get; set; }

        private void SetMagContinuous()
        {
            var reg3mData = (byte) 0b0000_0000;

            reg3mData |= Convert.ToByte(0x00); // continuous mode

            _magDevice.WriteAddressByte(CTRL_REG3_M, reg3mData);
        }

        private void SetAccContinuous()
        {
            var reg5xlData = (byte) 0b0000_0000;
            reg5xlData |= 0b0011_1000; // enable all 3 axes
            _accGyroDevice.WriteAddressByte(CTRL_REG5_XL, reg5xlData);

            var reg6xlData = (byte) 0b0000_0000;
            // 952Hz out data rate, BW set by ODR, 408Hz anti-aliasing
            reg6xlData |= (byte) AccelerometerDataRate._952Hz;
            _accGyroDevice.WriteAddressByte(CTRL_REG6_XL, reg6xlData);
        }

        private void SetGyroContinuous()
        {
            var regData = (byte) 0b0000_0000;
            regData |= (byte) 0b1100_0000;

            _accGyroDevice.WriteAddressByte(CTRL_REG1_G, regData);
        }

        private void SoftResetRebootMagnetometer()
        {
            var regData = (byte) 0b0000_0000;
            regData |= (byte) 0b0000_1100;

            _magDevice.WriteAddressByte(CTRL_REG2_M, regData);
        }

        private void SoftResetRebootAccGyro()
        {
            var regData = (byte) 0b0000_0000;
            regData |= (byte) 0b0000_0101;

            _accGyroDevice.WriteAddressByte(CTRL_REG8, regData);
        }

        public LSM9DS1(bool nothing)
        {
            Pi.Init<BootstrapWiringPi>();

            Console.WriteLine(Pi.Info.ToString());

            _accGyroDevice = Pi.I2C.AddDevice(AccGyroAddress);

            // Soft reset and reboot acc/gyro. Sets the IF_ADD_INC bit to 1 to auto-increment register addresses while reading multiple bytes in one go
            _accGyroDevice.WriteAddressByte(CTRL_REG8, 0b0100_0101);

            Thread.Sleep(10);

            CheckIdentity();

            _accGyroDevice.WriteAddressByte(CTRL_REG1_G, 0b0010_0000);

            // Selective axis enabling
            _accGyroDevice.WriteAddressByte(CTRL_REG4, 0b0011_1000);

            // Bypass FIFO mode selection
            _accGyroDevice.WriteAddressByte(FIFO_CTRL, 0b1100_0000);

            // Calibrate Acc
            CalibrateAcc();
            CalibrateGyro();
        }

        const int SampleCount = 50;
        float accXOffset = 0.0f;
        float accYOffset = 0.0f;
        float accZOffset = 0.0f;

        float accXVar = 0.0f;
        float accYVar = 0.0f;
        float accZVar = 0.0f;

        private void CalibrateAcc()
        {
            var xCumulated = 0.0f;
            var yCumulated = 0.0f;
            var zCumulated = 0.0f;

            for (var i = 0; i < SampleCount; i++)
            {
                var values = GetAccRaw();
                xCumulated += values.X;
                yCumulated += values.Y;
                zCumulated += values.Z;
            }

            accXOffset = xCumulated / SampleCount;
            accYOffset = yCumulated / SampleCount;
            accZOffset = zCumulated / SampleCount;

            var xVarCumulated = 0.0f;
            var yVarCumulated = 0.0f;
            var zVarCumulated = 0.0f;
            for (var i = 0; i < SampleCount; i++)
            {
                var values = GetAccRaw();
                xVarCumulated += Math.Abs(values.X - accXOffset);
                yVarCumulated += Math.Abs(values.Y - accYOffset);
                zVarCumulated += Math.Abs(values.Z - accZOffset);
            }

            accXVar = xVarCumulated / SampleCount;
            accYVar = yVarCumulated / SampleCount;
            accZVar = zVarCumulated / SampleCount;

            Console.WriteLine($"Acc offsets X {accXOffset} Y {accYOffset} Z {accZOffset}");
            Console.WriteLine($"Acc variance X {accXVar} Y {accYVar} Z {accZVar}");
        }

        float gyroXOffset = 0.0f;
        float gyroYOffset = 0.0f;
        float gyroZOffset = 0.0f;

        float gyroXVar = 0.0f;
        float gyroYVar = 0.0f;
        float gyroZVar = 0.0f;

        private void CalibrateGyro()
        {
            var xCumulated = 0.0f;
            var yCumulated = 0.0f;
            var zCumulated = 0.0f;

            for (var i = 0; i < SampleCount; i++)
            {
                var values = GetGyroRaw();
                xCumulated += values.X;
                yCumulated += values.Y;
                zCumulated += values.Z;
            }

            gyroXOffset = xCumulated / SampleCount;
            gyroYOffset = yCumulated / SampleCount;
            gyroZOffset = zCumulated / SampleCount;

            var xVarCumulated = 0.0f;
            var yVarCumulated = 0.0f;
            var zVarCumulated = 0.0f;
            for (var i = 0; i < SampleCount; i++)
            {
                var values = GetGyroRaw();
                xVarCumulated += Math.Abs(values.X - gyroXOffset);
                yVarCumulated += Math.Abs(values.Y - gyroYOffset);
                zVarCumulated += Math.Abs(values.Z - gyroZOffset);
            }

            gyroXVar = xVarCumulated / SampleCount;
            gyroYVar = yVarCumulated / SampleCount;
            gyroZVar = zVarCumulated / SampleCount;

            Console.WriteLine($"Gyro offsets X {gyroXOffset} Y {gyroYOffset} Z {gyroZOffset}");
            Console.WriteLine($"Gyro variance X {gyroXVar} Y {gyroYVar} Z {gyroZVar}");
        }

        private void CheckIdentity()
        {
            if (_magDevice.ReadAddressByte(WHO_AM_I_Mag) != MagIdentity)
            {
                throw new Exception("Could not find magnetometer. Check wiring!");
            }

            if (_accGyroDevice.ReadAddressByte(WHO_AM_I_AccGyro) != AccGyroIdentity)
            {
                throw new Exception("Could not find accelerometer/gyro. Check wiring!");
            }
        }

        CancellationTokenSource _cts;

        public event EventHandler<Vector3> GyroChanged = delegate { };
        public event EventHandler<Vector3> AccelerometerChanged = delegate { };

        public void StopInternalPolling()
        {
            _cts?.Cancel();
            _cts?.Dispose();
            _cts = null;
        }

        public void StartInternalPolling()
        {
            StopInternalPolling();

            _cts = new CancellationTokenSource();

            Task.Run(async () =>
            {
                var token = _cts.Token;
                while (!token.IsCancellationRequested)
                {
                    var currentGyroValues = ReadGyroRaw();
                    GyroChanged?.Invoke(this, currentGyroValues);

                    var currentAccValues = ReadAccRaw();
                    AccelerometerChanged?.Invoke(this, currentAccValues);

                    await Task.Delay(50);
                }
            });
        }

        public Vector3 ReadGyroRaw()
        {
            var uncorrectAccValues = GetGyroRaw();

            // Account for resting state values
            var x = uncorrectAccValues.X - gyroXOffset;
            var y = uncorrectAccValues.Y - gyroYOffset;
            var z = uncorrectAccValues.Z - gyroZOffset;

            x = Math.Abs(x) > gyroXVar ? x : 0.0f;
            y = Math.Abs(y) > gyroYVar ? y : 0.0f;
            z = Math.Abs(z) > gyroZVar ? z : 0.0f;

            return new Vector3(x, y, z);
        }

        public Vector3 ReadAccRaw()
        {
            var uncorrectAccValues = GetAccRaw();

            // Account for resting state values
            var x = uncorrectAccValues.X - accXOffset;
            var y = uncorrectAccValues.Y - accYOffset;
            var z = uncorrectAccValues.Z - accZOffset;

            x = Math.Abs(x) > accXVar ? x : 0.0f;
            y = Math.Abs(y) > accYVar ? y : 0.0f;
            z = Math.Abs(z) > accZVar ? z : 0.0f;

            return new Vector3(
                x,
                y,
                z
            );
        }

        private Vector3 GetGyroRaw()
        {
            var rawBytes = new byte[6];
            rawBytes[0] = _accGyroDevice.ReadAddressByte(OUT_X_L_G);
            rawBytes[1] = _accGyroDevice.ReadAddressByte(OUT_X_H_G);

            rawBytes[2] = _accGyroDevice.ReadAddressByte(OUT_Y_L_G);
            rawBytes[3] = _accGyroDevice.ReadAddressByte(OUT_Y_H_G);

            rawBytes[4] = _accGyroDevice.ReadAddressByte(OUT_Z_L_G);
            rawBytes[5] = _accGyroDevice.ReadAddressByte(OUT_Z_H_G);

            return AssembleValuesFromRawData(rawBytes);
        }

        private Vector3 GetAccRaw()
        {
            var rawBytes = new byte[6];
            rawBytes[0] = _accGyroDevice.ReadAddressByte(OUT_X_L_XL);
            rawBytes[1] = _accGyroDevice.ReadAddressByte(OUT_X_H_XL);

            rawBytes[2] = _accGyroDevice.ReadAddressByte(OUT_Y_L_XL);
            rawBytes[3] = _accGyroDevice.ReadAddressByte(OUT_Y_H_XL);

            rawBytes[4] = _accGyroDevice.ReadAddressByte(OUT_Z_L_XL);
            rawBytes[5] = _accGyroDevice.ReadAddressByte(OUT_Z_H_XL);

            return AssembleValuesFromRawData(rawBytes);
        }

        private Vector3 GetMagRaw()
        {
            var rawBytes = new byte[6];
            rawBytes[0] = _magDevice.ReadAddressByte(OUT_X_L_M);
            rawBytes[1] = _magDevice.ReadAddressByte(OUT_X_H_M);

            rawBytes[2] = _magDevice.ReadAddressByte(OUT_Y_L_M);
            rawBytes[3] = _magDevice.ReadAddressByte(OUT_Y_H_M);

            rawBytes[4] = _magDevice.ReadAddressByte(OUT_Z_L_M);
            rawBytes[5] = _magDevice.ReadAddressByte(OUT_Z_H_M);

            return AssembleValuesFromRawData(rawBytes);
        }

        private Vector3 AssembleValuesFromRawData(byte[] rawBytes)
        {
            var x = (((sbyte) rawBytes[1]) << 8) | (rawBytes[0] & 0xFF);
            var y = (((sbyte) rawBytes[3]) << 8) | (rawBytes[2] & 0xFF);
            var z = (((sbyte) rawBytes[5]) << 8) | (rawBytes[4] & 0xFF);

            return new Vector3(x, y, z);
        }
    }
}