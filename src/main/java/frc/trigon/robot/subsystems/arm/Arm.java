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
import org.littletonrobotics.junction.Logger;

public class Arm extends MotorSubsystem {
    private final TalonFXMotor
            armMasterMotor = ArmConstants.ARM_MASTER_MOTOR;
    private final CANcoderEncoder angleEncoder = ArmConstants.ANGLE_ENCODER;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ArmConstants.FOC_ENABLED);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            ArmConstants.ARM_DEFAULT_MAXIMUM_VELOCITY,
            ArmConstants.ARM_DEFAULT_MAXIMUM_ACCELERATION,
            ArmConstants.ARM_DEFAULT_MAXIMUM_JERK
    ).withEnableFOC(ArmConstants.FOC_ENABLED);
    public boolean isStateReversed = false;
    private ArmConstants.ArmState targetState = ArmConstants.ArmState.REST;

    public Arm() {
        setName("Arm");
    }

    @Override
    public SysIdRoutine.Config getSysIDConfig() {
        return ArmConstants.ARM_SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        armMasterMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        armMasterMotor.stopMotor();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Arm")
                .angularPosition(Units.Rotations.of(getAngle().getRotations()))
                .angularVelocity(Units.RotationsPerSecond.of(armMasterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(armMasterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void updateMechanism() {
        ArmConstants.ARM_MECHANISM.update(
                Rotation2d.fromRotations(getAngle().getRotations() + ArmConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET),
                Rotation2d.fromRotations(armMasterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) + ArmConstants.POSITION_OFFSET_FROM_GRAVITY_OFFSET)
        );
        Logger.recordOutput("Poses/Components/ArmPose", calculateVisualizationPose());
    }

    @Override
    public void updatePeriodically() {
        armMasterMotor.update();
        angleEncoder.update();
        Logger.recordOutput("Arm/CurrentPositionDegrees", getAngle().getDegrees());
    }

    @Override
    public void sysIDDrive(double targetVoltage) {
        armMasterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    public Pose3d calculateGamePieceCollectionPose() {
        return calculateVisualizationPose()
                .transformBy(ArmConstants.ARM_TO_HELD_GAME_PIECE);
    }

    public boolean isArmAboveSafeAngle() {
        return getAngle().getDegrees() >= ArmConstants.MAXIMUM_ARM_SAFE_ANGLE.getDegrees();
    }

    public boolean atState(ArmConstants.ArmState targetState, boolean isStateReversed) {
        if (isStateReversed)
            return this.targetState == targetState && atAngle(subtractFrom360Degrees(targetState.targetAngle));
        return atState(targetState);
    }

    public boolean atState(ArmConstants.ArmState targetState) {
        return this.targetState == targetState && atAngle(targetState.targetAngle);
    }

    public boolean atTargetAngle() {
        if (isStateReversed) {
            return atAngle(subtractFrom360Degrees(targetState.targetAngle));
        }
        return atAngle(targetState.targetAngle);
    }

    public boolean atPrepareAngle() {
        if (isStateReversed) {
            return atAngle(subtractFrom360Degrees(targetState.prepareAngle));
        }
        return atAngle(targetState.prepareAngle);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleEncoder.getSignal(CANcoderSignal.POSITION));
    }

    public Translation3d calculateLinearArmVelocity() {
        double velocityRotationsPerSecond = armMasterMotor.getSignal(TalonFXSignal.VELOCITY);
        double velocityMagnitude = velocityRotationsPerSecond * 2 * Math.PI * ArmConstants.ARM_LENGTH_METERS;
        return new Translation3d(
                getAngle().getCos() * velocityMagnitude,
                0,
                getAngle().getSin() * velocityMagnitude
        );
    }

    void setTargetState(ArmConstants.ArmState targetState) {
        this.isStateReversed = false;
        this.targetState = targetState;
        setTargetState(targetState, false);
    }

    void setTargetState(ArmConstants.ArmState targetState, boolean isStateReversed) {
        this.isStateReversed = isStateReversed;
        this.targetState = targetState;

        if (isStateReversed) {
            setTargetState(
                    subtractFrom360Degrees(targetState.targetAngle)
            );
            return;
        }
        setTargetState(
                targetState.targetAngle);
    }

    void setTargetState(Rotation2d targetAngle) {
        setTargetAngle(targetAngle);
    }

    void setPrepareState(ArmConstants.ArmState targetState) {
        setPrepareState(targetState, false);
    }

    void setPrepareState(ArmConstants.ArmState targetState, boolean isStateReversed) {
        this.isStateReversed = isStateReversed;
        this.targetState = targetState;

        if (isStateReversed) {
            setTargetAngle(subtractFrom360Degrees(targetState.prepareAngle));
            return;
        }
        setTargetAngle(targetState.prepareAngle);
    }

    void setArmState(ArmConstants.ArmState targetState) {
        setArmState(targetState, false);
    }

    void setArmState(ArmConstants.ArmState targetState, boolean isStateReversed) {
        this.isStateReversed = isStateReversed;
        this.targetState = targetState;

        if (isStateReversed) {
            setTargetAngle(subtractFrom360Degrees(targetState.targetAngle));
            return;
        }
        setTargetAngle(targetState.targetAngle);
    }

    private void setTargetAngle(Rotation2d targetAngle) {
        armMasterMotor.setControl(positionRequest.withPosition(Math.max(targetAngle.getRotations(), calculateMinimumArmSafeAngle().getRotations())));
    }

    private Rotation2d calculateMinimumArmSafeAngle() {
        final boolean isElevatorAboveSafeZone = RobotContainer.ELEVATOR.isElevatorAboveSafeZone();
        final double heightFromSafeZone = RobotContainer.ELEVATOR.getElevatorHeightFromMinimumSafeZone();
        final double cosOfMinimumSafeAngle = MathUtil.clamp(heightFromSafeZone / ArmConstants.ARM_LENGTH_METERS, 0, 1);
        return isElevatorAboveSafeZone
                ? Rotation2d.fromRadians(0)
                : Rotation2d.fromRadians(Math.acos(cosOfMinimumSafeAngle));
    }

    private boolean atAngle(Rotation2d targetAngle) {
        final double currentToTargetAngleDifferenceDegrees = Math.abs(targetAngle.minus(getAngle()).getDegrees());
        return currentToTargetAngleDifferenceDegrees < ArmConstants.ANGLE_TOLERANCE.getDegrees();
    }

    private static Rotation2d subtractFrom360Degrees(Rotation2d angleToSubtract) {
        return Rotation2d.fromDegrees(Rotation2d.k180deg.getDegrees() * 2 - angleToSubtract.getDegrees());
    }

    private Pose3d calculateVisualizationPose() {
        final Transform3d armTransform = new Transform3d(
                new Translation3d(0, 0, RobotContainer.ELEVATOR.getPositionMeters()),
                new Rotation3d(0, getAngle().getRadians(), 0)
        );
        return ArmConstants.ARM_VISUALIZATION_ORIGIN_POINT.transformBy(armTransform);
    }
}