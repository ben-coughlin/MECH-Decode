// Create this file: MathUtil.java
package com.example.ftcfieldsimulator; // Ensure this matches your package structure

public class MathUtil {

    /**
     * Clips a double value to be within a specified range [min, max].
     *
     * @param value The value to clip.
     * @param min   The minimum allowed value.
     * @param max   The maximum allowed value.
     * @return The clipped value.
     */
    public static double clip(double value, double min, double max) {
        if (min > max) {
            // This case should ideally not happen if logic is correct,
            // but handle it defensively.
            System.err.println("MathUtil.clip Warning: Min value (" + min + ") is greater than max value (" + max + "). Swapping them for clipping.");
            double temp = min;
            min = max;
            max = temp;
        }
        return Math.max(min, Math.min(value, max));
    }

    /**
     * Clips an integer value to be within a specified range [min, max].
     *
     * @param value The value to clip.
     * @param min   The minimum allowed value.
     * @param max   The maximum allowed value.
     * @return The clipped value.
     */
    public static int clip(int value, int min, int max) {
        if (min > max) {
            System.err.println("MathUtil.clip Warning: Min value (" + min + ") is greater than max value (" + max + "). Swapping them for clipping.");
            double temp = min;
            min = max;
            max = (int) temp; // Cast back if needed, though arguments are int
        }
        return Math.max(min, Math.min(value, max));
    }

    /**
     * Clips a long value to be within a specified range [min, max].
     *
     * @param value The value to clip.
     * @param min   The minimum allowed value.
     * @param max   The maximum allowed value.
     * @return The clipped value.
     */
    public static long clip(long value, long min, long max) {
        if (min > max) {
            System.err.println("MathUtil.clip Warning: Min value (" + min + ") is greater than max value (" + max + "). Swapping them for clipping.");
            long temp = min;
            min = max;
            max = temp;
        }
        return Math.max(min, Math.min(value, max));
    }

    // You can add clip methods for float if needed.
}
