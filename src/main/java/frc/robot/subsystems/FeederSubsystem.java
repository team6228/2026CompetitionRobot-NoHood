

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase{

    //private final VictorSP feedMotor = new VictorSP(FeederConstants.feederMotorPWM);
    private final VictorSP indexerMotor = new VictorSP(FeederConstants.indexerMotorPWM);

    private final SparkMax feedMotor = new SparkMax(12, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    
    public FeederSubsystem(){
        indexerMotor.setInverted(false);
        
    }
    

    public void feedShooter(){
        feedMotor.set(1);
        indexerMotor.set(1);
    }

    public void reverseFeed(){
        feedMotor.set(1);
        indexerMotor.set(-0.5);
    }

    public void stopMotors(){
        feedMotor.set(0);
        indexerMotor.set(0);
    }
}


