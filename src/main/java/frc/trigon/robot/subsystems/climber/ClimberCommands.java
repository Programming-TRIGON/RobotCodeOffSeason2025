package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import lib.commands.ExecuteEndCommand;
import lib.commands.NetworkTablesCommand;

import java.util.Set;
import java.util.function.DoubleSupplier;

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

    public static Command getSetTargetStateCommand(double targetPositionRotations, double targetServoPower, boolean isAffectedByRobotWeight) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetPositionRotations, targetServoPower, isAffectedByRobotWeight),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetSpeedCommand(DoubleSupplier targetSpeed) {
        return new ExecuteEndCommand(
                () -> RobotContainer.CLIMBER.setTargetVoltage(targetSpeed.getAsDouble() * ClimberConstants.MAXIMUM_MANUAL_CONTROL_VOLTAGE),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }
}
