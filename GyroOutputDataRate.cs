namespace dotnet.core.iot
{
    public enum GyroOutputDataRate
    {
        /// <summary>
        /// Power down
        /// </summary>
        PowerDown = 0b000_0_0000,

        /// <summary>
        /// 14.9Hz
        /// </summary>
        _14_9Hz = 0b001_0_0000,

        /// <summary>
        /// 59.5Hz
        /// </summary>
        _59_5Hz = 0b010_0_0000,

        /// <summary>
        /// 119Hz
        /// </summary>
        _119Hz = 0b011_0_0000,

        /// <summary>
        /// 238Hz
        /// </summary>
        _238Hz = 0b100_0_0000,

        /// <summary>
        /// 47Hz
        /// </summary>
        _476Hz = 0b101_0_0000,

        /// <summary>
        /// 952Hz
        /// </summary>
        _952Hz = 0b110_0_0000
    }
}