package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import lib.utilities.Conversions;

public class Elevator extends MotorSubsystem {
    private final TalonFXMotor masterMotor = ElevatorConstants.MASTER_MOTOR;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(0, ElevatorConstants.DEFAULT_MAXIMUM_VELOCITY, ElevatorConstants.DEFAULT_MAXIMUM_ACCELERATION, ElevatorConstants.DEFAULT_MAXIMUM_ACCELERATION * 10).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private ElevatorConstants.ElevatorState targetState = ElevatorConstants.ElevatorState.REST;

    public Elevator() {
        setName("Elevator");
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ElevatorConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        masterMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Elevator")
                .linearPosition(Units.Meters.of(getPositionRotations()))
                .linearVelocity(Units.MetersPerSecond.of(masterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    public void updateMechanism() {
        ElevatorConstants.MECHANISM.update(
                getPositionMeters(),
                rotationsToMeters(masterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );

        Logger.recordOutput("Poses/Components/ElevatorFirstPose", getFirstStageComponentPose());
        Logger.recordOutput("Poses/Components/ElevatorSecondPose", getSecondStageComponentPose());
    }

    @Override
    public void updatePeriodically() {
        masterMotor.update();
        Logger.recordOutput("Elevator/CurrentPositionMeters", getPositionMeters());
    }

    void setTargetState(ElevatorConstants.ElevatorState targetState) {
        this.targetState = targetState;
        scalePositionRequestSpeed(targetState.speedScalar);
        setTargetPositionRotations(metersToRotations(targetState.targetPositionMeters));
    }

    void setTargetPositionRotations(double targetPositionRotations) {
        masterMotor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    private Pose3d getFirstStageComponentPose() {
            return calculateCurrentElevatorPoseFromOrigin(ElevatorConstants.FIRST_STAGE_VISUALIZATION_ORIGIN_POINT);
    }

    private Pose3d getSecondStageComponentPose() {
            return calculateCurrentElevatorPoseFromOrigin(ElevatorConstants.SECOND_STAGE_VISUALIZATION_ORIGIN_POINT)
    }

    private Pose3d calculateCurrentElevatorPoseFromOrigin(Pose3d originPoint) {
        return calculateComponentPose(originPoint, getPositionMeters());
    }

    private Pose3d calculateComponentPose(Pose3d originPoint, double poseHeight) {
        final Transform3d elevatorTransform = new Transform3d(
                new Translation3d(0, 0, poseHeight),
                new Rotation3d()
        );
        return originPoint.transformBy(elevatorTransform);
    }

    private void scalePositionRequestSpeed(double speedScalar) {
        positionRequest.Velocity = ElevatorConstants.DEFAULT_MAXIMUM_VELOCITY * speedScalar;
        positionRequest.Acceleration = ElevatorConstants.DEFAULT_MAXIMUM_ACCELERATION * speedScalar;
        positionRequest.Jerk = positionRequest.Acceleration * 10;
    }

    private double getPositionMeters() {
        return rotationsToMeters(getPositionRotations());
    }

    private double rotationsToMeters(double positionsRotations) {
        return Conversions.rotationsToDistance(positionsRotations, ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double metersToRotations(double positionsMeters) {
        return Conversions.distanceToRotations(positionsMeters, ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double getPositionRotations() {
        return masterMotor.getSignal(TalonFXSignal.POSITION);
    }
}