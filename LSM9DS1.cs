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

        private const byte CTRL_REG1_G = 0b00010000;
        private const byte CTRL_REG2_G = 0b00010001;
        private const byte CTRL_REG3_G = 0b00010010;

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

        private const byte CTRL_REG8 = 0x22;

        static II2CDevice _accGyroDevice;

        public LSM9DS1 () {
            Pi.Init<BootstrapWiringPi> ();

            Console.WriteLine (Pi.Info.ToString ());

            _accGyroDevice = Pi.I2C.AddDevice (AccGyroAddress);

            // Soft reset and reboot acc/gyro. Sets the IF_ADD_INC bit to 1 to auto-increment register addresses while reading multiple bytes in one go
            _accGyroDevice.WriteAddressByte (CTRL_REG8, 0b0000_0101);

            _accGyroDevice.WriteAddressByte (CTRL_REG5_XL, 0b0000_1000);

            Thread.Sleep (10);

            CheckIdentity ();

            _accGyroDevice.WriteAddressByte (CTRL_REG1_G, 0b1000_0000);
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
                    var currentValues = ReadGyroRaw ();
                    GyroChanged?.Invoke (this, currentValues);

                    await Task.Delay (100);
                }
            });
        }

        public Vector3 ReadGyroRaw () {
            var rawBytes = new byte[6];
            rawBytes[0] = _accGyroDevice.ReadAddressByte (OUT_X_L_G);
            rawBytes[1] = _accGyroDevice.ReadAddressByte (OUT_X_H_G);
            rawBytes[2] = _accGyroDevice.ReadAddressByte (OUT_Y_L_G);
            rawBytes[3] = _accGyroDevice.ReadAddressByte (OUT_Y_H_G);
            rawBytes[4] = _accGyroDevice.ReadAddressByte (OUT_Z_L_G);
            rawBytes[5] = _accGyroDevice.ReadAddressByte (OUT_Z_H_G);

            return new Vector3 (
                (rawBytes[1] << 8) | rawBytes[0],
                (rawBytes[3] << 8) | rawBytes[2],
                (rawBytes[5] << 8) | rawBytes[4]
            );
        }
    }
}