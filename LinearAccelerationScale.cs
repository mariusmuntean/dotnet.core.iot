namespace dotnet.core.iot
{
    /// <summary>
    /// Linear Acceleration
    /// </summary>
    public enum LinearAccelerationScale
    {
        /// <summary>
        /// +/- 2G
        /// </summary>
        _2G = 0b000_00_000,

        /// <summary>
        /// +/- 4G
        /// </summary>
        _4G = 0b000_10_000,

        /// <summary>
        /// +/- 8G
        /// </summary>
        _8G = 0b000_11_000,

        /// <summary>
        /// +/- 16G
        /// </summary>
        _16G = 0b000_01_000
    }
}