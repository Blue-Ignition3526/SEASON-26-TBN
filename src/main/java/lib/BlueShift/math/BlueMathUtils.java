package lib.BlueShift.math;

public class BlueMathUtils {
    public static double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static double lerp(double a, double b, double t) {
        return (1-t)*a + t*b;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(Math.min(value, max), min);
    }
}
