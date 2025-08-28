package frc.trigon.robot.commands.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;

public class ClimbCommands {
    public static Command getClimbCommand() {
        return new SequentialCommandGroup(
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.PREPARE_FOR_CLIMB)
                        .until(RobotContainer.CLIMBER::hasCage),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB)
                        .until(RobotContainer.CLIMBER::atTargetState),
                ClimberCommands.getSetTargetSpeedCommand(OperatorConstants.DRIVER_CONTROLLER.getRightY())
        );
    }
}
