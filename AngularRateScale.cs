namespace dotnet.core.iot
{
    public enum AngularRateScale
    {
        /// <summary>
        /// +/- 245dps
        /// </summary>
        _245 = 0b000_00_000,

        /// <summary>
        /// +/-500 dps
        /// </summary>
        _500 = 0b000_01_000,

        /// <summary>
        /// +/-2000 dps
        /// </summary>
        _2000 = 0b000_11_000
    }
}