package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    
    double distError = this.bandCenter - distance;	// compute the difference between how far the robot is from the wall and it's desired position
    
    if(Math.abs(distError) <= this.bandwidth) {          // if the position of the robot falls within the margin of error, maintain the same speed 
    	WallFollowingLab.leftMotor.setSpeed(motorHigh); 
        WallFollowingLab.rightMotor.setSpeed(motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    	 	
    }
    else if(distError<0) {
    	WallFollowingLab.leftMotor.setSpeed(motorLow); // if the robot is too far from the wall, change the motor speed's so the robot fixes it's position
        WallFollowingLab.rightMotor.setSpeed(motorHigh);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    }
    else if(distError>0) {
    	WallFollowingLab.leftMotor.setSpeed(motorHigh); // if the robot is too close to the wall, change the motor speed's so the robot fixes it's position
        WallFollowingLab.rightMotor.setSpeed(motorLow);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    }
    
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
