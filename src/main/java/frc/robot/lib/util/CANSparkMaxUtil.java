package frc.robot.lib.util;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

// import edu.wpi.first.wpilibj.AnalogInput;
// import frc.robot.lib.util.CANSparkMaxUtil.Usage;

public class CANSparkMaxUtil {
    public enum Usage {
        kAll,
        kPositionOnly,
        kVelocityOnly,
        kMinimal;
    }

    public static final Usage kMinimal = null;

public static void setCANSparkMaxBusUsage(
    CANSparkMax motor, Usage usage, boolean enableFollowing) {
    if (enableFollowing) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    } else {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    }

    if (usage == Usage.kAll) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
    } else if (usage == Usage.kPositionOnly) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    }

    if (usage == Usage.kVelocityOnly) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    }

    if (usage == Usage.kMinimal) {
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    }

    }

    public static void setCANSparkMaxBusUsage(CANSparkMax angleMotor, Usage usage){
        setCANSparkMaxBusUsage(angleMotor, usage, false);

    }

}
