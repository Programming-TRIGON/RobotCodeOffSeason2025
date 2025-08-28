package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import trigon.commands.ExecuteEndCommand;
import trigon.commands.NetworkTablesCommand;

import java.util.Set;

public class ClimberCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetPositionRotations, targetServoSpeed) -> RobotContainer.CLIMBER.setTargetState(targetPositionRotations, targetServoSpeed, false),
                false,
                Set.of(RobotContainer.CLIMBER),
                "Debugging/TargetPositionRotations",
                "Debugging/TargetServoSpeed"
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetState),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(double targetPositionRotations, double targetServoSpeed, boolean affectedByRobotWeight) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetPositionRotations, targetServoSpeed, affectedByRobotWeight),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetSpeedCommand(double targetSpeed) {
        return new ExecuteEndCommand(
                () -> RobotContainer.CLIMBER.setTargetVoltage(targetSpeed * ClimberConstants.MAXIMUM_MANUAL_CONTROL_VOLTAGE),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }
}
