package ca.team4308.absolutelib.math;

public class DoubleUtils {
    public static double normalize(double d){
        if(d > 1.0){
            return 1.0;
        }
        if(d < -1.0){
            return -1.0;
        } else {
            return d;
        }
    }

    public static double clamp(double d, double min, double max){
        if(d > max){
            return max;
        }
        else if(d < min){
            return min;
        } else {
            return d;
        }
    }

    public static double mapRange(double d, double old_min, double old_max, double new_min, double new_max) {
        return (new_min + (new_max - new_min) * (d - old_min) / (old_max - old_min));
    }

    public static double mapRangeNew(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
