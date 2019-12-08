using System;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using Unosquare.RaspberryIO;
using Unosquare.RaspberryIO.Abstractions;
using Unosquare.WiringPi;

namespace dotnet.core.iot {
    public class LSM9DS1 {
        private const byte AccGyroAddress = 0x6B;
        private const byte MagAddress = 0x6B;

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

        public LSM9DS1 () {
            Pi.Init<BootstrapWiringPi> ();

            Console.WriteLine (Pi.Info.ToString ());

            _accGyroDevice = Pi.I2C.AddDevice (AccGyroAddress);

            // Soft reset and reboot acc/gyro. Sets the IF_ADD_INC bit to 1 to auto-increment register addresses while reading multiple bytes in one go
            _accGyroDevice.WriteAddressByte (CTRL_REG8, 0b0100_0101);

            Thread.Sleep (10);

            CheckIdentity ();

            _accGyroDevice.WriteAddressByte (CTRL_REG1_G, 0b0010_0000);

            // Selective axis enabling
            _accGyroDevice.WriteAddressByte (CTRL_REG4, 0b0011_1000);

            // Bypass FIFO mode selection
            _accGyroDevice.WriteAddressByte (FIFO_CTRL, 0b1100_0000);

            // Calibrate Acc
            CalibrateAcc ();
            CalibrateGyro ();
        }

        const int SampleCount = 50;
        float accXOffset = 0.0f;
        float accYOffset = 0.0f;
        float accZOffset = 0.0f;

        float accXVar = 0.0f;
        float accYVar = 0.0f;
        float accZVar = 0.0f;

        private void CalibrateAcc () {
            var xCumulated = 0.0f;
            var yCumulated = 0.0f;
            var zCumulated = 0.0f;

            for (int i = 0; i < SampleCount; i++) {
                var values = GetAccRaw ();
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
            for (int i = 0; i < SampleCount; i++) {
                var values = GetAccRaw ();
                xVarCumulated += Math.Abs (values.X - accXOffset);
                yVarCumulated += Math.Abs (values.Y - accYOffset);
                zVarCumulated += Math.Abs (values.Z - accZOffset);
            }
            accXVar = xVarCumulated / SampleCount;
            accYVar = yVarCumulated / SampleCount;
            accZVar = zVarCumulated / SampleCount;

            Console.WriteLine ($"Acc offsets X {accXOffset} Y {accYOffset} Z {accZOffset}");
            Console.WriteLine ($"Acc variance X {accXVar} Y {accYVar} Z {accZVar}");
        }

        float gyroXOffset = 0.0f;
        float gyroYOffset = 0.0f;
        float gyroZOffset = 0.0f;

        float gyroXVar = 0.0f;
        float gyroYVar = 0.0f;
        float gyroZVar = 0.0f;

        private void CalibrateGyro () {
            var xCumulated = 0.0f;
            var yCumulated = 0.0f;
            var zCumulated = 0.0f;

            for (int i = 0; i < SampleCount; i++) {
                var values = GetGyroRaw ();
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
            for (int i = 0; i < SampleCount; i++) {
                var values = GetGyroRaw ();
                xVarCumulated += Math.Abs (values.X - gyroXOffset);
                yVarCumulated += Math.Abs (values.Y - gyroYOffset);
                zVarCumulated += Math.Abs (values.Z - gyroZOffset);
            }
            gyroXVar = xVarCumulated / SampleCount;
            gyroYVar = yVarCumulated / SampleCount;
            gyroZVar = zVarCumulated / SampleCount;

            Console.WriteLine ($"Gyro offsets X {gyroXOffset} Y {gyroYOffset} Z {gyroZOffset}");
            Console.WriteLine ($"Gyro variance X {gyroXVar} Y {gyroYVar} Z {gyroZVar}");
        }

        private void CheckIdentity () {
            // if (_magDevice.ReadRegister(WHO_AM_I_Mag) != MagIdentity)
            // {
            //     throw new Exception("Could not find magnetometer. Check wiring!");
            // }

            if (_accGyroDevice.ReadAddressByte (WHO_AM_I_AccGyro) != AccGyroIdentity) {
                throw new Exception ("Could not find accelerometer/gyro. Check wiring!");
            }
        }

        CancellationTokenSource _cts;

        public event EventHandler<Vector3> GyroChanged = delegate { };
        public event EventHandler<Vector3> AccelerometerChanged = delegate { };

        public void StopInternalPolling () {
            _cts?.Cancel ();
            _cts?.Dispose ();
            _cts = null;
        }

        public void StartInternalPolling () {
            StopInternalPolling ();

            _cts = new CancellationTokenSource ();

            Task.Run (async () => {
                var token = _cts.Token;
                while (!token.IsCancellationRequested) {
                    var currentGyroValues = ReadGyroRaw ();
                    GyroChanged?.Invoke (this, currentGyroValues);

                    var currentAccValues = ReadAccRaw ();
                    AccelerometerChanged?.Invoke (this, currentAccValues);

                    await Task.Delay (50);
                }
            });
        }

        public Vector3 ReadGyroRaw () {
            var uncorrectAccValues = GetGyroRaw ();

            // Account for resting state values
            float x = uncorrectAccValues.X - gyroXOffset;
            float y = uncorrectAccValues.Y - gyroYOffset;
            float z = uncorrectAccValues.Z - gyroZOffset;

            x = Math.Abs (x) > gyroXVar?x : 0.0f;
            y = Math.Abs (y) > gyroYVar?y : 0.0f;
            z = Math.Abs (z) > gyroZVar?z : 0.0f;

            return new Vector3 (x, y, z);
        }

        public Vector3 ReadAccRaw () {

            var uncorrectAccValues = GetAccRaw ();

            // Account for resting state values
            float x = uncorrectAccValues.X - accXOffset;
            float y = uncorrectAccValues.Y - accYOffset;
            float z = uncorrectAccValues.Z - accZOffset;

            x = Math.Abs (x) > accXVar ? x : 0.0f;
            y = Math.Abs (y) > accYVar ? y : 0.0f;
            z = Math.Abs (z) > accZVar ? z : 0.0f;

            return new Vector3 (
                x,
                y,
                z
            );
        }

        private Vector3 GetGyroRaw () {
            var rawBytes = new byte[6];
            rawBytes[0] = _accGyroDevice.ReadAddressByte (OUT_X_L_G);
            rawBytes[1] = _accGyroDevice.ReadAddressByte (OUT_X_H_G);

            rawBytes[2] = _accGyroDevice.ReadAddressByte (OUT_Y_L_G);
            rawBytes[3] = _accGyroDevice.ReadAddressByte (OUT_Y_H_G);

            rawBytes[4] = _accGyroDevice.ReadAddressByte (OUT_Z_L_G);
            rawBytes[5] = _accGyroDevice.ReadAddressByte (OUT_Z_H_G);

            return AssembleValuesFromRawData (rawBytes);
        }

        private Vector3 GetAccRaw () {
            var rawBytes = new byte[6];
            rawBytes[0] = _accGyroDevice.ReadAddressByte (OUT_X_L_XL);
            rawBytes[1] = _accGyroDevice.ReadAddressByte (OUT_X_H_XL);

            rawBytes[2] = _accGyroDevice.ReadAddressByte (OUT_Y_L_XL);
            rawBytes[3] = _accGyroDevice.ReadAddressByte (OUT_Y_H_XL);

            rawBytes[4] = _accGyroDevice.ReadAddressByte (OUT_Z_L_XL);
            rawBytes[5] = _accGyroDevice.ReadAddressByte (OUT_Z_H_XL);

            return AssembleValuesFromRawData (rawBytes);
        }

        private Vector3 AssembleValuesFromRawData (byte[] rawBytes) {

            int x = (((sbyte) rawBytes[1]) << 8) | (rawBytes[0] & 0xFF);
            int y = (((sbyte) rawBytes[3]) << 8) | (rawBytes[2] & 0xFF);
            int z = (((sbyte) rawBytes[5]) << 8) | (rawBytes[4] & 0xFF);

            return new Vector3 (x, y, z);
        }
    }
}