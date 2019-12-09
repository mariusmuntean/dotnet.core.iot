using System;
using System.Device.I2c;
using System.Threading;
using System.Threading.Tasks;
using dotnet.core.iot.csharp.AHRS;
using Grpc.Net.Client;
using GrpcGreeter;
using Iot.Device.Lsm9Ds1;

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

        //    static void Main(string[] args)
        //    {
        //        var lsm9DS1 = new LSM9DS1();

        //        lsm9DS1.Begin();

        //        while (true)
        //        {
        //            var acc = lsm9DS1.ReadAccelerometer();
        //            var gyro = lsm9DS1.ReadGyroscope();
        //            var mag = lsm9DS1.ReadMagnetometer();

        //            Console.WriteLine($"Accelerometer x {acc.X:N3}   y {acc.X:N3}   z {acc.Z:N3}");
        //            Console.WriteLine($"Gyroscope     x {gyro.X:N3}  y {gyro.X:N3}  z {gyro.Z:N3}");
        //            Console.WriteLine($"Magnetometer  x {mag.X:N3}   y {mag.X:N3}   z {mag.Z:N3}");
        //            Console.WriteLine();

        //            Thread.Sleep(50);
        //        }

        //        Thread.Sleep(Timeout.Infinite);
        //    }

        static async Task Main(string[] args)
        {
            // This switch must be set before creating the GrpcChannel/HttpClient.
            AppContext.SetSwitch(
                "System.Net.Http.SocketsHttpHandler.Http2UnencryptedSupport", true);

            var channel = GrpcChannel.ForAddress("http://192.168.0.40:5000");
            var client = new Location.LocationClient(channel);

            var degreesToRadiansFactor = Math.PI / 180;
            var radiansToDegreesFactor = 180.0f / Math.PI;
            var samplePeriod = TimeSpan.FromMilliseconds(50);

            // var ahrs = new MahonyAHRS(samplePeriod, 5f);
            var ahrs = new MadgwickAHRS((float) samplePeriod.TotalSeconds, 0.1f);
            var lsm9DS1 = new LSM9DS1();

            lsm9DS1.Begin();

            while (true)
            {
                // read fresh sensor data
                var acc = lsm9DS1.ReadAccelerometer();
                var gyro = lsm9DS1.ReadGyroscope();
                var mag = lsm9DS1.ReadMagnetometer();

                // convert gyro readings from dps to rad/s
                var gx = (float) (gyro.X * degreesToRadiansFactor);
                var gy = (float) (gyro.Y * degreesToRadiansFactor);
                var gz = (float) (gyro.Z * degreesToRadiansFactor);

                // update ahrs
                ahrs.Update(gx, gy, gz,
                    acc.X, acc.Y, acc.Z,
                    mag.X, mag.Y, mag.Z
                );

                ahrs.ComputeAngles();

                // output rol, pitch and yaw
                var rollDegrees = ahrs.Roll * radiansToDegreesFactor;
                var pitchDegrees = ahrs.Pitch * radiansToDegreesFactor;
                var yawDegrees = ahrs.Yaw * radiansToDegreesFactor;

                client.UpdateLocationAsync(
                    new NewLocation {Roll = rollDegrees, Pitch = pitchDegrees, Yaw = yawDegrees}
                );

                Console.WriteLine($"Roll    {rollDegrees:N3}    Pitch {pitchDegrees:N3}    Yaw {yawDegrees:N3}");
                Console.WriteLine();

                Thread.Sleep(samplePeriod);
            }

            Thread.Sleep(Timeout.Infinite);
        }

        //        public const int I2cAddress = 0x6B;
        //        static void Main (string[] args) {
        //            using (var ag = new Lsm9Ds1AccelerometerAndGyroscope (CreateI2cDevice ())) {
        //                while (true) {
        //
        //                    Console.WriteLine ($"Accelerometer x {ag.Acceleration.X:N3}  y {ag.Acceleration.X:N3}  z {ag.Acceleration.Z:N3}");
        //                    Console.WriteLine ($"Gyroscope     x {ag.AngularRate.X:N3}   y {ag.AngularRate.X:N3}   z {ag.AngularRate.Z:N3}");
        //
        //                    Thread.Sleep (100);
        //                }
        //            }
        //        }
        //
        //        private static I2cDevice CreateI2cDevice () {
        //            var settings = new I2cConnectionSettings (1, I2cAddress);
        //            return I2cDevice.Create (settings);
        //        }

        // static void Main(string[] args)
        // {
        //     const double degreesToRadiansFactor = Math.PI / 180;
        //     const double radiansToDegreesFactor = 180.0f / Math.PI;
        //     const int samplePeriod = 50;

        //     var ahrs = new MahonyAHRS(samplePeriod, 5f);
        //     // var ahrs = new MadgwickAHRS(samplePeriod, 0.1f);

        //     using (var ag = new Lsm9Ds1AccelerometerAndGyroscope(CreateI2cDevice(), 
        //         angularRateScale: Iot.Device.Lsm9Ds1.AngularRateScale.Scale2000Dps))
        //     {
        //         while (true)
        //         {
        //             // convert gyro readings from dps to rad/s
        //             var gx = (float) (ag.AngularRate.X * degreesToRadiansFactor);
        //             var gy = (float) (ag.AngularRate.Y * degreesToRadiansFactor);
        //             var gz = (float) (ag.AngularRate.Z * degreesToRadiansFactor);

        //             // update ahrs
        //             ahrs.Update(gx, gy, gz,
        //                 ag.Acceleration.X, ag.Acceleration.Y, ag.Acceleration.Z
        //             );

        //             ahrs.ComputeAngles();

        //             // output rol, pitch and yaw
        //             var rollDegrees = ahrs.Roll * radiansToDegreesFactor;
        //             var pitchDegrees = ahrs.Pitch * radiansToDegreesFactor;
        //             var yawDegrees = ahrs.Yaw * radiansToDegreesFactor;
        //             Console.WriteLine($"Roll    {rollDegrees:N3}    Pitch {pitchDegrees:N3}    Yaw {yawDegrees:N3}");
        //             Console.WriteLine();

        //             Thread.Sleep(samplePeriod);

        //             Thread.Sleep(100);
        //         }
        //     }
        // }
        // public const int I2cAddress = 0x6B;
        // private static I2cDevice CreateI2cDevice()
        // {
        //     var settings = new I2cConnectionSettings(1, I2cAddress);
        //     return I2cDevice.Create(settings);
        // }
    }
}