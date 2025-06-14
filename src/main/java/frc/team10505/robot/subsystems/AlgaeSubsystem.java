package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team10505.robot.Constants.HardwareConstants.*;
import static frc.team10505.robot.Constants.AlgaeConstants.*;

public class AlgaeSubsystem extends SubsystemBase {
    // Variables
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private PIDController pivotController;
    private ArmFeedforward pivotFeedForward;
    public static boolean coasting = false;

    // Constants
    private static final int kPivotMotorCurrentLimit = 1;
    private static final double pivotEncoderScale = 1;
    private static double pivotSetpoint = 0;
    private static double intakeSpeed = 0;
    private static double absoluteOffset = 180;
    private static double startingAngle = -90;
    private static double simSpeed = 0;

    // Motor Definitions
    private SparkMax pivotMotor = new SparkMax(ALGAE_PIVOT_MOTOR_ID, MotorType.kBrushless);
    private SparkMax intakeMotor = new SparkMax(ALGAE_INTAKE_MOTOR_ID, MotorType.kBrushless);
    private AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

    // Motor Controllers
    private SparkMaxConfig IntakeMotorConfig = new SparkMaxConfig();

    // Constructor
    public AlgaeSubsystem() {
        /* Pivot Configurator */
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(kPivotMotorCurrentLimit, kPivotMotorCurrentLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale);

        if (Utils.isSimulation()) {
            pivotController = new PIDController(0, 0, 0);
            pivotFeedForward = new ArmFeedforward(0, 0, 0, 0);
        } else {
            pivotController = new PIDController(0.1, 0, 0);
            pivotFeedForward = new ArmFeedforward(0, 0.08, 0, 0);
        }
    }

    // Things That Do Things (Funktchins)
    public double getEffort() {
        return pivotFeedForward
                .calculate(Units.degreesToRadians(getPivotEncoder()), 0)
                + pivotController.calculate(getPivotEncoder(), pivotSetpoint);
    }

    public double getPivotEncoder() {
        return (-pivotEncoder.getPosition() + absoluteOffset);
    }

    public Command setAngle(double Angle) {
        return run(() -> {
            pivotSetpoint = Angle;
        });
    }

    public Command setVoltage(double Voltage) {
        return run(() -> {
            pivotMotor.setVoltage(Voltage);
        });
    }

    public Command runIntake(double speed) {
        if (Utils.isSimulation()) {
            return runEnd(() -> {
                simSpeed = speed;
            }, () -> {
                simSpeed = 0;
            });
        } else {
            return runEnd(() -> {
                intakeMotor.set(speed);
            }, () -> {
                intakeMotor.set(0);
            });
        }
    }

    public Boolean hasAlgae() {
        return pivotMotor.getOutputCurrent() > 20;
    }

    // Periodic Stuff
    @Override
    public void periodic() {
        // dashboard stuff
        SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
        SmartDashboard.putNumber("Pivot Encoder", getPivotEncoder());
        SmartDashboard.putNumber("Pivot Calculated Effort", getEffort());
        SmartDashboard.putNumber("Algae Intake Speed", intakeSpeed);
        SmartDashboard.putNumber("Algae Intake Motor Output", intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());
        SmartDashboard.putNumber("pivotEncoder", getPivotEncoder());
        SmartDashboard.putBoolean("Has Algae", hasAlgae());
        pivotMotor.setVoltage(getEffort());
    }
}
