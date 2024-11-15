package frc.robot.util;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class MathUtils {

    public static double round(double number, int newScale) {
        return new BigDecimal(number).setScale(newScale, RoundingMode.HALF_UP).doubleValue();
    }

}
