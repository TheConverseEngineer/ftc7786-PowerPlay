package org.firstinspires.ftc.teamcode.utils;

public class MathUtils {
    // Suppress instantiation
    private MathUtils() { }

    public static double clamp(double x, double min, double max) {
        return Math.max(Math.min(x, max), min);
    }
}
