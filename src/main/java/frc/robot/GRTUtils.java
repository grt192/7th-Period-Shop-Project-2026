package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.units.measure.Angle;

public class GRTUtils {
    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static double mapJoystick(double x, double outMin, double outMax) {
        return map(x, -1, 1, outMin, outMax);
    }

    public static Angle mapJoystick(double x, Angle outMin, Angle outMax) {
        return Degrees.of(map(x, -1, 1, outMin.in(Degrees), outMax.in(Degrees)));
    }

    // Source - https://stackoverflow.com/a
    // Posted by Jonik, modified by community. See post 'Timeline' for change
    // history
    // Retrieved 2025-12-03, License - CC BY-SA 4.0
    public static double round(double value, int places) {
        if (places < 0)
            throw new IllegalArgumentException();

        BigDecimal bd = BigDecimal.valueOf(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

}
