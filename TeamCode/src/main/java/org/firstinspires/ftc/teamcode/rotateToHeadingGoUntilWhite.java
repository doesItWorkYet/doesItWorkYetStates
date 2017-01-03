/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="rotateAndGoToHeading", group="Testing")  // @Autonomous(...) is the other common choice
@Disabled
public class rotateToHeadingGoUntilWhite extends LinearOpMode {
    HardwareMapLucyV4 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        //Update Telemetry with initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new HardwareMapLucyV4();
        robot.init(hardwareMap);
        robot.zero();
        //wait for good orientation data
        while(robot.orientation.getOrientation()[0] == 0);
        //zero
        robot.setZeroHeading(robot.orientation.getOrientation()[0]);
        //
        //Wait for start and reset the runtime count
        double headingToApproach = robot.ADVANCE_TO_BEACON_HEADING;
        double currentHeading;
        boolean hasFinishedTurn = false;
        waitForStart();
       while(opModeIsActive()){
            currentHeading = robot.orientation.getOrientation()[0];
            if(Math.abs(headingToApproach - currentHeading) <= robot.HEADING_ACCURACY){
                robot.stop();
                hasFinishedTurn = true;
                moveUntilWhite();
            }

           else{
                double directionToTurn = robot.decideDriectionToTurn(currentHeading, headingToApproach);
                if(Math.abs(headingToApproach - currentHeading) <= 20){
                    if(directionToTurn < 0) {
                        robot.turn(-robot.ROTATION_TURNING_SPEED/2.0);
                    }
                    if(directionToTurn > 0){
                        robot.turn(robot.ROTATION_TURNING_SPEED/2.0);
                    }
                }
                if(directionToTurn < 0) {
                   robot.turn(-robot.ROTATION_TURNING_SPEED);
                }
                if(directionToTurn > 0){
                    robot.turn(robot.ROTATION_TURNING_SPEED);
                }
            }

           idle();
       }

    }
    public void moveUntilWhite(){

    }

}

