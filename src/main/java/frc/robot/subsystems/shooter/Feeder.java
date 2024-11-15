package frc.robot.subsystems.shooter;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import frc.robot.subsystems.single_motor.SingleMotorIo;
import frc.robot.subsystems.single_motor.SingleMotorSubsystem;

public class Feeder extends SingleMotorSubsystem {
    public Feeder(SingleMotorIo io) {
        super("Feeder", io, CONSTANTS.getFeederForwardVelocity(), CONSTANTS.getFeederReverseVelocity());
    }
}
