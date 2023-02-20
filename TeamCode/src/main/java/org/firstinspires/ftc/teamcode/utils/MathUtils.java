package org.firstinspires.ftc.teamcode.utils;

public class MathUtils {
    // Suppress instantiation
    private MathUtils() { }

    public static double clamp(double x, double min, double max) {
        return Math.max(Math.min(x, max), min);
    }

    /** Returns true if two doubles are within 0.00001 of each other */
    public static boolean epsEquals(double a, double b) {
        return Math.abs(a - b) < 0.00001;
    }

    public static double wrap(double x) {
        while (x < -Math.PI) x+= 2*Math.PI;
        while (x > Math.PI) x-= 2*Math.PI;
        return x;
    }
}
