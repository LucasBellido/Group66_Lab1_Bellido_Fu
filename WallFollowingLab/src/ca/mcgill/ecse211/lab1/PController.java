package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
    
    int pTypeConstant = 15;
    double distError = this.bandCenter - distance;	// compute the difference between how far the robot is from the wall and it's desired position
    
    if(Math.abs(distError) <= this.bandwidth) {          // if the position of the robot falls within the margin of error, maintain the same speed  	
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); 
        WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    	 	
    }
    else if(distError<0) {
    	int correctionSpeed = Math.abs(pTypeConstant*distError);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-correctionSpeed); // if the robot is too far from the wall, change the motor speed's so the robot fixes it's position
        WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+correctionSpeed);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    }
    else if(distError>0) {
    	int correctionSpeed = pTypeConstant*distError;
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+correctionSpeed); // if the robot is too close to the wall, change the motor speed's so the robot fixes it's position
        WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED-correctionSpeed);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    }
    
    
    
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
