package org.firstinspires.ftc.simteamcode;

public class SystemClock {
    /**
     * Returns the current time in milliseconds, similar to how uptime might be used
     * in other contexts. This is equivalent to System.currentTimeMillis().
     *
     * @return The difference, measured in milliseconds, between the current time
     *         and midnight, January 1, 1970 UTC.
     */
    public static long uptimeMillis() {
        return System.currentTimeMillis();
    }
}
