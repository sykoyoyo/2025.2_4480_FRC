package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  private final SparkFlex armLeader;
  private final SparkFlex armFollower;
  private final SparkFlex intakeFlex;
  private final PIDController armPID;


  private static final double kP = 0.05;
  private static final double kI = 0.0;
  private static final double kD = 0.001;

  private static final double[] armPositions = {0.0, 90.0, 45.0, 20.0, -15.0};

  public ArmSubsystem() {
    armLeader = new SparkFlex(9, MotorType.kBrushless);
    armFollower = new SparkFlex(10, MotorType.kBrushless);
    intakeFlex = new SparkFlex(12, MotorType.kBrushless);

    armPID = new PIDController(kP, kI, kD);
    armPID.setTolerance(2.0);

    SparkFlexConfig armLeaderConfig = new SparkFlexConfig();
    SparkFlexConfig armFollowerConfig = new SparkFlexConfig();
    SparkFlexConfig intakeFlexConfig = new SparkFlexConfig();
    SparkFlexConfig globalConfig = new SparkFlexConfig();

    globalConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    armLeaderConfig
      .apply(globalConfig)
      .inverted(true);

    armFollowerConfig
      .apply(globalConfig)
      .inverted(true);
    
    intakeFlexConfig
      .apply(globalConfig);
  
    armLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armFollower.configure(armFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setArmPower(double power) {
    armLeader.set(power);
  }

  public void moveToPosition(int positionIndex) {
    if (positionIndex <0 || positionIndex>= armPositions.length) return;
    double setpoint = armPositions[positionIndex];
    double currentPos = getArmPosition();
    double output = armPID.calculate(currentPos, setpoint);
    armLeader.set(output);
  }
  public void moveArm(double power){
    armLeader.set(power);
    armFollower.set(-power);
  }
  
  public void coralIntake(double power){
  intakeFlex.set(power);
}
  public void stopCoral() {
  intakeFlex.set(0);
}

  public double getArmPosition() {
    return armLeader.getEncoder().getPosition();
  }

  public void stopArm() {
    armLeader.set(0);
    armFollower.set(0);
  }

@Override
public void periodic() { 
  SmartDashboard.putNumber("Arm Position", getArmPosition());
}

}