package frc.robot.util;

import java.util.function.BooleanSupplier;

public class SupplierUtil {
    /**
     * Invert boolean supplier
     * 
     * @param booleanSupplier Supplier to be inverted
     * @return inverted Supplier
     */
    public static BooleanSupplier not(BooleanSupplier booleanSupplier) {
        return () -> !booleanSupplier.getAsBoolean();
    }

    public static BooleanSupplier xor(BooleanSupplier supplier1, BooleanSupplier supplier2) {
        return () -> supplier1.getAsBoolean() != supplier2.getAsBoolean();
    }
}
