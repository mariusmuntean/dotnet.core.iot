using System;
using System.Threading;

namespace dotnet.core.iot
{
    class Program
    {
//        static void Main (string[] args) {
//
//            var lsm9DS1 = new LSM9DS1 (false);
//            lsm9DS1.GyroChanged += (sender, vector3) =>
//            {
//                Console.WriteLine($"Gyro X: {vector3.X} Y: {vector3.Y} Z: {vector3.Z}");
//            };
//            lsm9DS1.AccelerometerChanged += (sender, vector3) => {
//                Console.WriteLine ($"Accelerometer  X: {vector3.X} Y: {vector3.Y} Z: {vector3.Z}");
//            };
//
//            lsm9DS1.StartInternalPolling ();
//
//            Thread.Sleep (Timeout.Infinite);
//        }

        static void Main(string[] args)
        {
            var lsm9DS1 = new LSM9DS1();

            lsm9DS1.Begin();

            while (true)
            {
                var acc = lsm9DS1.ReadAccelerometer();
                var gyro = lsm9DS1.ReadGyroscope();
                var mag = lsm9DS1.ReadMagnetometer();

                Console.WriteLine($"Accelerometer x {acc.X:N3}  y {acc.X:N3}  z {acc.Z:N3}");
                Console.WriteLine($"Gyroscope x {gyro.X:N3}  y {gyro.X:N3}  z {gyro.Z:N3}");
                Console.WriteLine($"Magnetometer x {mag.X:N3}  y {mag.X:N3}  z {mag.Z:N3}");
                Console.WriteLine();

                Thread.Sleep(50);
            }

            Thread.Sleep(Timeout.Infinite);
        }
    }
}