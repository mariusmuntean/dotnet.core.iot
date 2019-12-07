using System;
using System.Threading;

namespace dotnet.core.iot
{
    class Program
    {
        static void Main(string[] args)
        {
            
            var lsm9DS1 = new LSM9DS1();
            lsm9DS1.GyroChanged += (sender, vector3) =>
            {
                Console.WriteLine($"X: {vector3.X} Y: {vector3.Y} Z: {vector3.Z}");
            };
            
            
            lsm9DS1.StartInternalPolling();

            Thread.Sleep(Timeout.Infinite);
        }
    }
}