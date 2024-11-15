package frc.robot.subsystems.shooter;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import frc.robot.subsystems.single_motor.SingleMotorIo;
import frc.robot.subsystems.single_motor.SingleMotorSubsystem;

public class Intake extends SingleMotorSubsystem {
    public Intake(SingleMotorIo io) {
        super("Shooter/Intake", io, CONSTANTS.getIntakeForwardVelocity(), CONSTANTS.getIntakeReverseVelocity());
    }
}
