package frc.robot.subsystems.climber;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import frc.robot.subsystems.single_motor.SingleMotorIo;
import frc.robot.subsystems.single_motor.SingleMotorSubsystem;

public class Traverser extends SingleMotorSubsystem {
    public Traverser(SingleMotorIo io) {
        super("Traverser", io, CONSTANTS.getTraverserFowardVelocity(), CONSTANTS.getTraverserFowardVelocity());
    }
}
