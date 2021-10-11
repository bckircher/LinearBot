package frc.robot;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class testTrapezoid {
  @Test
  void trapezoid() {
    TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1),
                           new TrapezoidProfile.State(7, 0),
                           new TrapezoidProfile.State(0, 0));

    for(int i = 0; ; i++) {
      TrapezoidProfile.State setpoint = profile.calculate(i / 10.0);
      System.out.println((i / 10.0) + "," + setpoint.position + "," + setpoint.velocity);
      if(profile.isFinished(i / 10.0)) {
        break;
      }
    }
  }

  @Test
  void feedforward() {
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.Drive.kS, Constants.Drive.kV, Constants.Drive.kA);

    for(int i = 0; i <= 100; i++) {
      System.out.println((i / 10.0) + ": " + ff.calculate(i / 10.0));
    }
  }
}
