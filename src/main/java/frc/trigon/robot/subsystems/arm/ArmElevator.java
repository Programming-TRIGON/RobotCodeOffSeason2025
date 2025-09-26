package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.MotorSubsystem;
import lib.hardware.phoenix6.cancoder.CANcoderEncoder;
import lib.hardware.phoenix6.cancoder.CANcoderSignal;
import lib.hardware.phoenix6.talonfx.TalonFXMotor;
import lib.hardware.phoenix6.talonfx.TalonFXSignal;
import lib.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class ArmElevator extends MotorSubsystem {
    private final TalonFXMotor
            armMasterMotor = ArmElevatorConstants.ARM_MASTER_MOTOR,
            elevatorMasterMotor = ArmElevatorConstants.ELEVATOR_MASTER_MOTOR;
    private final CANcoderEncoder angleEncoder = ArmElevatorConstants.ANGLE_ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ArmElevatorConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage armPositionRequest = new DynamicMotionMagicVoltage(
            0,
            ArmElevatorConstants.ARM_DEFAULT_MAXIMUM_VELOCITY,
            ArmElevatorConstants.ARM_DEFAULT_MAXIMUM_ACCELERATION,
            ArmElevatorConstants.ARM_DEFAULT_MAXIMUM_JERK
    ).withEnableFOC(ArmElevatorConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            ArmElevatorConstants.ELEVATOR_DEFAULT_MAXIMUM_VELOCITY,
            ArmElevatorConstants.ELEVATOR_DEFAULT_MAXIMUM_ACCELERATION,
            ArmElevatorConstants.ELEVATOR_DEFAULT_MAXIMUM_ACCELERATION * 10
    ).withEnableFOC(ArmElevatorConstants.FOC_ENABLED);
    public boolean isStateReversed = false;
    private ArmElevatorConstants.ArmElevatorState targetState = ArmElevatorConstants.ArmElevatorState.REST;

    public ArmElevator() {
        setName("ArmElevator");
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ArmElevatorConstants.ARM_SYSID_CONFIG;
        //return ArmElevatorConstants.ELEVATOR_SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        armMasterMotor.setBrake(brake);
        elevatorMasterMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        armMasterMotor.stopMotor();
        elevatorMasterMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Arm")
                .angularPosition(Units.Rotations.of(getCurrentArmAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(armMasterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(armMasterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
//        log.motor("Elevator")
//                .linearPosition(Units.Meters.of(getPositionRotations()))
//                .linearVelocity(Units.MetersPerSecond.of(masterMotor.getSignal(TalonFXSignal.VELOCITY)))
//                .voltage(Units.Volts.of(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        ArmElevatorConstants.ARM_MECHANISM.update(
                Rotation2d.fromRotations(getCurrentArmAngle().getRotations() + ArmElevatorConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET),
                Rotation2d.fromRotations(armMasterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) + ArmElevatorConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET)
        );
        ArmElevatorConstants.ELEVATOR_MECHANISM.update(
                getCurrentElevatorPositionMeters(),
                rotationsToMeters(elevatorMasterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );

        Logger.recordOutput("Poses/Components/ArmPose", calculateVisualizationPose());
        Logger.recordOutput("Poses/Components/ElevatorFirstPose", getFirstStageComponentPose());
        Logger.recordOutput("Poses/Components/ElevatorSecondPose", getSecondStageComponentPose());
    }

    @Override
    public void updatePeriodically() {
        armMasterMotor.update();
        angleEncoder.update();
        elevatorMasterMotor.update();
        Logger.recordOutput("Elevator/CurrentPositionMeters", getCurrentElevatorPositionMeters());
        Logger.recordOutput("Arm/CurrentPositionDegrees", getCurrentArmAngle().getDegrees());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        armMasterMotor.setControl(voltageRequest.withOutput(targetVoltage));
//        elevatorMasterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    public Pose3d calculateGamePieceCollectionPose() {
        return calculateVisualizationPose()
                .transformBy(ArmElevatorConstants.ARM_TO_HELD_GAME_PIECE);
    }

    public boolean isArmAboveSafeAngle() {
        return getCurrentArmAngle().getDegrees() >= ArmElevatorConstants.MAXIMUM_ARM_SAFE_ANGLE.getDegrees();
    }

    public boolean atTargetState() {
        return atState(targetState, isStateReversed);
    }

    public boolean atState(ArmElevatorConstants.ArmElevatorState targetState) {
        return atState(targetState, false);
    }

    public boolean atState(ArmElevatorConstants.ArmElevatorState targetState, boolean isStateReversed) {
        if (targetState == null)
            return false;
        return armAtAngle(isStateReversed ? subtractFrom360Degrees(targetState.targetAngle) : targetState.targetAngle)
                && elevatorAtPosition(targetState.targetPositionMeters);
    }

    public boolean armAtAngle(Rotation2d targetAngle) {
        final double currentToTargetAngleDifferenceDegrees = Math.abs(targetAngle.minus(getCurrentArmAngle()).getDegrees());
        return currentToTargetAngleDifferenceDegrees < ArmElevatorConstants.ANGLE_TOLERANCE.getDegrees();
    }

    public boolean elevatorAtPosition(double positionMeters) {
        final double currentToTargetStateDifferenceMeters = Math.abs(positionMeters - getCurrentElevatorPositionMeters());
        return currentToTargetStateDifferenceMeters < ArmElevatorConstants.ELEVATOR_POSITION_TOLERANCE_METERS;
    }

    public Rotation2d getCurrentArmAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION));
    }

    public double getCurrentElevatorPositionMeters() {
        return rotationsToMeters(getElevatorPositionRotations());
    }

    public Translation3d calculateLinearArmVelocity() {
        double velocityRotationsPerSecond = armMasterMotor.getSignal(TalonFXSignal.VELOCITY);
        double velocityMagnitude = velocityRotationsPerSecond * 2 * Math.PI * ArmElevatorConstants.ARM_LENGTH_METERS;
        return new Translation3d(
                getCurrentArmAngle().getCos() * velocityMagnitude,
                0,
                getCurrentArmAngle().getSin() * velocityMagnitude
        );
    }

    void prepareToState(ArmElevatorConstants.ArmElevatorState targetState) {
        setTargetState(targetState.prepareState);
    }

    void prepareToState(ArmElevatorConstants.ArmElevatorState targetState, boolean isStateReversed) {
        if (targetState.prepareState == null) {
            setTargetState(targetState, isStateReversed);
            return;
        }
        setTargetState(targetState.prepareState, isStateReversed);
    }

    void setTargetState(ArmElevatorConstants.ArmElevatorState targetState) {
        setTargetState(targetState, false);
    }

    void setTargetState(ArmElevatorConstants.ArmElevatorState targetState, boolean isStateReversed) {
        this.isStateReversed = isStateReversed;
        this.targetState = targetState;

        setTargetArmState(targetState, isStateReversed);
        setTargetElevatorState(targetState);
    }

    void setTargetArmState(ArmElevatorConstants.ArmElevatorState targetState, boolean isStateReversed) {
        if (isStateReversed) {
            setTargetArmAngle(subtractFrom360Degrees(targetState.targetAngle), targetState.ignoreConstraints);
            return;
        }
        setTargetArmAngle(targetState.targetAngle, targetState.ignoreConstraints);
    }

    void setTargetElevatorState(ArmElevatorConstants.ArmElevatorState targetState) {
        scaleElevatorPositionRequestSpeed(targetState.speedScalar);
        setTargetElevatorPositionMeters(targetState.targetPositionMeters, targetState.ignoreConstraints);
    }

    void setTargetArmAngle(Rotation2d targetAngle, boolean ignoreConstraints) {
        armMasterMotor.setControl(armPositionRequest.withPosition(ignoreConstraints ? targetAngle.getRotations() : Math.max(targetAngle.getRotations(), calculateMinimumArmSafeAngle().getRotations())));
    }

    void setTargetElevatorPositionMeters(double targetPositionMeters, boolean ignoreConstraints) {
        setTargetElevatorPositionRotations(metersToRotations(targetPositionMeters), ignoreConstraints);
    }

    void setTargetElevatorPositionRotations(double targetPositionRotations, boolean ignoreConstraints) {
        elevatorMasterMotor.setControl(positionRequest.withPosition(ignoreConstraints ? targetPositionRotations : Math.max(targetPositionRotations, calculateMinimumSafeElevatorHeightRotations())));
    }

    private Rotation2d calculateMinimumArmSafeAngle() {
        final double heightFromSafeZone = getElevatorHeightFromMinimumSafeZone();
        final double cosOfMinimumSafeAngle = MathUtil.clamp(heightFromSafeZone / ArmElevatorConstants.ARM_LENGTH_METERS, 0, 1);
        final double acos = Math.acos(cosOfMinimumSafeAngle);
        return Double.isNaN(acos)
                ? Rotation2d.fromRadians(0)
                : Rotation2d.fromRadians(acos);
    }

    private double calculateMinimumSafeElevatorHeightRotations() {
        final double armCos = RobotContainer.ARM_ELEVATOR.getCurrentArmAngle().getCos();
        final double elevatorHeightFromArm = armCos * ArmElevatorConstants.ARM_LENGTH_METERS;
        final double minimumElevatorSafeZone = ArmElevatorConstants.MINIMUM_ELEVATOR_SAFE_ZONE_METERS;
        final double minimumSafeHeightMeters = (RobotContainer.ARM_ELEVATOR.isArmAboveSafeAngle()
                ? 0 : elevatorHeightFromArm)
                + minimumElevatorSafeZone;
        return metersToRotations(minimumSafeHeightMeters);
    }

    private double getElevatorHeightFromMinimumSafeZone() {
        return getCurrentElevatorPositionMeters() - ArmElevatorConstants.MINIMUM_ELEVATOR_SAFE_ZONE_METERS;
    }

    private static Rotation2d subtractFrom360Degrees(Rotation2d angleToSubtract) {
        return Rotation2d.fromDegrees(Rotation2d.k180deg.getDegrees() * 2 - angleToSubtract.getDegrees());
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d armTransform = new Transform3d(
                new Translation3d(0, 0, getCurrentElevatorPositionMeters()),
                new Rotation3d(0, getCurrentArmAngle().getRadians(), 0)
        );
        return ArmElevatorConstants.ARM_VISUALIZATION_ORIGIN_POINT.transformBy(armTransform);
    }

    private void scaleElevatorPositionRequestSpeed(double speedScalar) {
        positionRequest.Velocity = ArmElevatorConstants.ELEVATOR_DEFAULT_MAXIMUM_VELOCITY * speedScalar;
        positionRequest.Acceleration = ArmElevatorConstants.ELEVATOR_DEFAULT_MAXIMUM_ACCELERATION * speedScalar;
        positionRequest.Jerk = positionRequest.Acceleration * 10;
    }

    private Pose3d getFirstStageComponentPose() {
        return calculateComponentPose(ArmElevatorConstants.ELEVATOR_FIRST_STAGE_VISUALIZATION_ORIGIN_POINT, getFirstStageComponentPoseHeight());
    }

    private Pose3d getSecondStageComponentPose() {
        return calculateComponentPose(ArmElevatorConstants.ELEVATOR_SECOND_STAGE_VISUALIZATION_ORIGIN_POINT, getCurrentElevatorPositionMeters());
    }

    private double getFirstStageComponentPoseHeight() {
        if (isSecondStageComponentLimitReached())
            return getCurrentElevatorPositionMeters() - ArmElevatorConstants.SECOND_ELEVATOR_COMPONENT_EXTENDED_LENGTH_METERS;
        return 0;
    }

    private boolean isSecondStageComponentLimitReached() {
        return getCurrentElevatorPositionMeters() > ArmElevatorConstants.SECOND_ELEVATOR_COMPONENT_EXTENDED_LENGTH_METERS;
    }

    private Pose3d calculateComponentPose(Pose3d originPoint, double currentHeight) {
        final Transform3d elevatorTransform = new Transform3d(
                new Translation3d(0, 0, currentHeight),
                new Rotation3d()
        );
        return originPoint.transformBy(elevatorTransform);
    }


    private double rotationsToMeters(double positionsRotations) {
        return Conversions.rotationsToDistance(positionsRotations, ArmElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double metersToRotations(double positionsMeters) {
        return Conversions.distanceToRotations(positionsMeters, ArmElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double getElevatorPositionRotations() {
        return elevatorMasterMotor.getSignal(TalonFXSignal.POSITION);
    }
}