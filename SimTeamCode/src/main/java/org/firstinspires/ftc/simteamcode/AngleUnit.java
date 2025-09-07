package org.firstinspires.ftc.simteamcode;

@SuppressWarnings("WeakerAccess")
public enum AngleUnit
{
    DEGREES(0), RADIANS(1);
    public final byte bVal;

    protected static final double TwoPi   = 2 * Math.PI;
    public    static final float  Pif     = (float) Math.PI;

    AngleUnit(int i)
    {
        bVal = (byte) i;
    }

    //----------------------------------------------------------------------------------------------
    // Primitive operations
    //----------------------------------------------------------------------------------------------

    public double fromDegrees(double degrees)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return this.normalize(degrees / 180.0 * Math.PI);
            case DEGREES:                   return this.normalize(degrees);
        }
    }

    public float fromDegrees(float degrees)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return this.normalize(degrees / 180.0f * Pif);
            case DEGREES:                   return this.normalize(degrees);
        }
    }

    public double fromRadians(double radians)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return this.normalize(radians);
            case DEGREES:                   return this.normalize(radians / Math.PI * 180.0);
        }
    }

    public float fromRadians(float radians)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return this.normalize(radians);
            case DEGREES:                   return this.normalize(radians / Pif * 180.0f);
        }
    }

    public double fromUnit(AngleUnit them, double theirs)
    {
        switch (them)
        {
            default:
            case RADIANS:                   return this.fromRadians(theirs);
            case DEGREES:                   return this.fromDegrees(theirs);
        }
    }

    public float fromUnit(AngleUnit them, float theirs)
    {
        switch (them)
        {
            default:
            case RADIANS:                   return this.fromRadians(theirs);
            case DEGREES:                   return this.fromDegrees(theirs);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Derived operations
    //----------------------------------------------------------------------------------------------

    public double toDegrees(double inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return DEGREES.fromRadians(inOurUnits);
            case DEGREES:                   return DEGREES.fromDegrees(inOurUnits);
        }
    }

    public float toDegrees(float inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return DEGREES.fromRadians(inOurUnits);
            case DEGREES:                   return DEGREES.fromDegrees(inOurUnits);
        }
    }

    public double toRadians(double inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return RADIANS.fromRadians(inOurUnits);
            case DEGREES:                   return RADIANS.fromDegrees(inOurUnits);
        }
    }

    public float toRadians(float inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:                   return RADIANS.fromRadians(inOurUnits);
            case DEGREES:                   return RADIANS.fromDegrees(inOurUnits);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Normalization
    //----------------------------------------------------------------------------------------------

    public double normalize(double mine)
    {
        switch (this)
        {
            default:
            case RADIANS:               return normalizeRadians(mine);
            case DEGREES:               return normalizeDegrees(mine);
        }
    }

    public float normalize(float mine)
    {
        switch (this)
        {
            default:
            case RADIANS:               return normalizeRadians(mine);
            case DEGREES:               return normalizeDegrees(mine);
        }
    }

    public static double normalizeDegrees(double degrees)
    {
        if (Math.abs(degrees) > 720.0)
        {
            double numRevolutions = Math.floor(Math.abs(degrees) / 360.0);
            degrees = Math.signum(degrees) * (Math.abs(degrees) - (numRevolutions * 360.0));
        }
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    public static float normalizeDegrees(float degrees)
    {
        return (float)normalizeDegrees((double)degrees);
    }

    public static double normalizeRadians(double radians)
    {
        if (Math.abs(radians) > 4 * Math.PI)
        {
            double numRevolutions = Math.floor(Math.abs(radians) / (2*Math.PI));
            radians = Math.signum(radians) * (Math.abs(radians) - (numRevolutions * (2*Math.PI)));
        }
        while (radians >= Math.PI) radians -= TwoPi;
        while (radians < -Math.PI) radians += TwoPi;
        return radians;
    }

    public static float normalizeRadians(float radians)
    {
        return (float)normalizeRadians((double)radians);
    }

    public UnnormalizedAngleUnit getUnnormalized()
    {
        switch (this)
        {
            default:
            case RADIANS:   return UnnormalizedAngleUnit.RADIANS;
            case DEGREES:   return UnnormalizedAngleUnit.DEGREES;
        }
    }

}
