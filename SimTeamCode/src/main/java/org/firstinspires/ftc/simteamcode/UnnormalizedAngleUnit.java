package org.firstinspires.ftc.simteamcode;

@SuppressWarnings("WeakerAccess")
public enum UnnormalizedAngleUnit
{
    DEGREES(0), RADIANS(1);
    public final byte bVal;

    UnnormalizedAngleUnit(int i)
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
            case RADIANS:  return (degrees / 180.0 * Math.PI);
            case DEGREES:  return (degrees);
        }
    }

    public float fromDegrees(float degrees)
    {
        switch (this)
        {
            default:
            case RADIANS:  return (degrees / 180.0f * AngleUnit.Pif);
            case DEGREES:  return (degrees);
        }
    }

    public double fromRadians(double radians)
    {
        switch (this)
        {
            default:
            case RADIANS:  return (radians);
            case DEGREES:  return (radians / Math.PI * 180.0);
        }
    }

    public float fromRadians(float radians)
    {
        switch (this)
        {
            default:
            case RADIANS:  return (radians);
            case DEGREES:  return (radians / AngleUnit.Pif * 180.0f);
        }
    }

    public double fromUnit(UnnormalizedAngleUnit them, double theirs)
    {
        switch (them)
        {
            default:
            case RADIANS:  return this.fromRadians(theirs);
            case DEGREES:  return this.fromDegrees(theirs);
        }
    }

    public float fromUnit(UnnormalizedAngleUnit them, float theirs)
    {
        switch (them)
        {
            default:
            case RADIANS:  return this.fromRadians(theirs);
            case DEGREES:  return this.fromDegrees(theirs);
        }
    }

    public double fromUnit(AngleUnit them, double theirs) {
        return this.fromUnit(them.getUnnormalized(), theirs);
    }

    public float fromUnit(AngleUnit them, float theirs) {
        return this.fromUnit(them.getUnnormalized(), theirs);
    }

    //----------------------------------------------------------------------------------------------
    // Derived operations
    //----------------------------------------------------------------------------------------------

    public double toDegrees(double inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:      return DEGREES.fromRadians(inOurUnits);
            case DEGREES:      return DEGREES.fromDegrees(inOurUnits);
        }
    }

    public float toDegrees(float inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:      return DEGREES.fromRadians(inOurUnits);
            case DEGREES:      return DEGREES.fromDegrees(inOurUnits);
        }
    }

    public double toRadians(double inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:      return RADIANS.fromRadians(inOurUnits);
            case DEGREES:      return RADIANS.fromDegrees(inOurUnits);
        }
    }

    public float toRadians(float inOurUnits)
    {
        switch (this)
        {
            default:
            case RADIANS:      return RADIANS.fromRadians(inOurUnits);
            case DEGREES:      return RADIANS.fromDegrees(inOurUnits);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Normalization
    //----------------------------------------------------------------------------------------------

    public AngleUnit getNormalized()
    {
        switch (this)
        {
            default:
            case RADIANS:  return AngleUnit.RADIANS;
            case DEGREES:  return AngleUnit.DEGREES;
        }
    }
}
