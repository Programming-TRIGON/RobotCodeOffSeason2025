package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class IntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetVoltage, targetAngleDegrees) -> RobotContainer.INTAKE.setTargetState(targetVoltage, Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                Set.of(RobotContainer.INTAKE),
                "Debugging/IntakeTargetAngleDegrees",
                "Debugging/IntakeTargetVoltage"
        );
    }

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetTargetStateCommand(double targetVoltage, Rotation2d targetAngle) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetVoltage, targetAngle),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }
}