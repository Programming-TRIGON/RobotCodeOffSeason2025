package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import lib.commands.NetworkTablesCommand;

import java.util.Set;

public class ElevatorCommands {
    public static Command getDebbugingCommand() {
        return new NetworkTablesCommand(
                RobotContainer.ELEVATOR::setTargetPositionRotations,
                false,
                Set.of(RobotContainer.ELEVATOR),
                "Debugging/ElevatorTargetPositionRotations"
        );
    }

    public static Command getSetTargetStateCommand(ElevatorConstants.ElevatorState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetState(targetState),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }
    
    public static Command getSetTargetStateCommand(double targetPositionRotations) {
        return new StartEndCommand(
                () -> RobotContainer.ELEVATOR.setTargetPositionRotations(targetPositionRotations),
                RobotContainer.ELEVATOR::stop,
                RobotContainer.ELEVATOR
        );
    }
}