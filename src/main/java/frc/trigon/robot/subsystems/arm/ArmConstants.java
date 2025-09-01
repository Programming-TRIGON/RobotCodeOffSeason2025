package frc.trigon.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import trigon.hardware.RobotHardwareStats;
import trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import trigon.hardware.simulation.SingleJointedArmSimulation;
import trigon.utilities.mechanisms.SingleJointedArmMechanism2d;

public class ArmConstants {
    private static final int
            MASTER_MOTOR_ID = 13,
            FOLLOWER_MOTOR_ID = 14,
            ENCODER_ID = 13;
    private static final String
            MASTER_MOTOR_NAME = "ArmMasterMotor",
            FOLLOWER_MOTOR_NAME = "ArmFollowerMotor",
            ENCODER_NAME = "ArmEncoder";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME);

    private static final double GEAR_RATIO = 50;
    private static final double ANGLE_ENCODER_GRAVITY_OFFSET = 0;
    private static final boolean SHOULD_FOLLOWER_OPPOSE_MASTER = false;
    static final double
            DEFAULT_MAXIMUM_VELOCITY = RobotHardwareStats.isSimulation() ? 0 : 0,
            DEFAULT_MAXIMUM_ACCELERATION = RobotHardwareStats.isSimulation() ? 0 : 0,
            DEFAULT_MAXIMUM_JERK = DEFAULT_MAXIMUM_ACCELERATION * 10;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double
            ARM_LENGTH_METERS = 0.67,
            ARM_MASS_KILOGRAMS = 3.5;
    private static final Rotation2d
            ARM_MINIMUM_ANGLE = new Rotation2d(0),
            ARM_MAXIMUM_ANGLE = new Rotation2d(359);
    private static final boolean SHOULD_SIMULATE_GRAVITY = true;
    private static final SingleJointedArmSimulation SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            ARM_LENGTH_METERS,
            ARM_MASS_KILOGRAMS,
            ARM_MINIMUM_ANGLE,
            ARM_MAXIMUM_ANGLE,
            SHOULD_SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Seconds),
            Units.Volts.of(1),
            Units.Second.of(1000)
    );

    static final Pose3d ARM_VISUALIZATION_ORIGIN_POINT = new Pose3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
    );

    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            "ArmMechanism",
            ARM_LENGTH_METERS,
            Color.kBlue
    );

    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0);

    static {
        configureMasterMotor();
        configureFollowerMotor();
        configureEncoder();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.RotorToSensorRatio = GEAR_RATIO;

        config.Feedback.FeedbackRemoteSensorID = MASTER_MOTOR.getID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.kP = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kI = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kD = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kS = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kV = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kA = RobotHardwareStats.isSimulation() ? 0 : 0;
        config.Slot0.kG = RobotHardwareStats.isSimulation() ? 0 : 0;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = DEFAULT_MAXIMUM_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = DEFAULT_MAXIMUM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_MOTOR.getID(), SHOULD_FOLLOWER_OPPOSE_MASTER);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }

    private static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = ANGLE_ENCODER_GRAVITY_OFFSET;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MASTER_MOTOR);

        ENCODER.registerSignal(CANcoderSignal.POSITION, 100);
        ENCODER.registerSignal(CANcoderSignal.VELOCITY, 100);
    }

    public enum ArmState {
        REST(Rotation2d.fromDegrees(0)),
        L1(Rotation2d.fromDegrees(90)),
        L2(Rotation2d.fromDegrees(110)),
        L3(Rotation2d.fromDegrees(110)),
        L4(Rotation2d.fromDegrees(165)),
        NET(Rotation2d.fromDegrees(160)),
        PROCESSOR(Rotation2d.fromDegrees(50));

        public final Rotation2d targetAngle;

        ArmState(Rotation2d targetAngle) {
            this.targetAngle = targetAngle;
        }
    }
}