package org.firstinspires.ftc.teamcode;

public class Units {

    private static final double FEET_TO_METERS_FACTOR = 0.3048;
    private static final double METERS_TO_CENTIMETERS_FACTOR = 100;

    public static double CENTIMETERS = 0;

    /**
     * Converts a measurement in feet to meters.
     *
     * @param feet The length in feet.
     * @return The equivalent length in meters.
     */
    public static double feetToMeters(double feet) {
        return feet * FEET_TO_METERS_FACTOR;
    }

    /**
     * Converts a measurement in meters to centimeters.
     *
     * @param meters The length in meters.
     * @return The equivalent length in centimeters.
     */
    public static double metersToCentimeters(double meters) {
        return meters * METERS_TO_CENTIMETERS_FACTOR;
    }

    /**
     * Converts a measurement in feet to centimeters.
     *
     * @param feet The length in feet.
     * @return The equivalent length in centimeters.
     */
    public static double feetToCentimeters(double feet) {
        return metersToCentimeters(feetToMeters(feet));
    }

    /**
     * Converts distance to centimeters if in meters.
     *
     * @param distanceInMeters The distance in meters.
     * @param isTargetInCm Flag indicating if target is in centimeters.
     * @return The equivalent distance in centimeters or meters.
     */
    public static double convertToCentimeters(double distanceInMeters, boolean isTargetInCm) {
        if (isTargetInCm) {
            return distanceInMeters * METERS_TO_CENTIMETERS_FACTOR;  // Convert meters to cm
        } else {
            return distanceInMeters;  // Keep it in meters if target distance is in meters
        }
    }

    // Add more conversion methods as needed...
        public static double METER = 1;

        public static double CENTIMETER = 100;


}
