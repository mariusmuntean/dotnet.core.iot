namespace dotnet.core.iot
{
    public enum MagneticSensitivityScale
    {
        /// <summary>
        /// +/- 4 Gauss
        /// </summary>
        _4Gauss = 0b0_00_00000,

        /// <summary>
        /// +/- 8 Gauss
        /// </summary>
        _8Gauss = 0b0_01_00000,

        /// <summary>
        /// +/- 12 Gauss
        /// </summary>
        _12Gauss = 0b0_10_00000,

        /// <summary>
        /// +/- 16 Gauss
        /// </summary>
        _16Gauss = 0b0_11_00000
    }
}