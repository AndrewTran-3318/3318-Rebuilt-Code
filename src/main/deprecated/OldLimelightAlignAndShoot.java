public class OldLimelightAlignAndShoot {
// ---- Combination of limelight code and using april tag targetting directly ----
            
            // if (Math.abs(LimelightHelpers.getTX(Constants.LIMELIGHT_NAME)) < 3.0 && shooter.isAtSpeed(targetSpeed)) {
            //     state = State.FEED;
            // }

            //limelight code
            // drivetrain.setControl(
            //     drive.withVelocityX(0) 
            //         .withVelocityY(0) 
            //         .withRotationalRate(1)
            //         //.withRotationalRate(LimelightHelpers.getTX(Constants.LIMELIGHT_NAME) * -Constants.TX_ROTATIONAL_SPEED) // Locks rotation towards april tag
            // );

            //boolean hasTarget = LimelightHelpers.getTV(Constants.LIMELIGHT_NAME); //Does the limelight see a target (TV)
            // var results = camera.getAllUnreadResults();
            // boolean hasTarget = false;
            // PhotonTrackedTarget target = null;
            // for (var result : results) {
            //     if (result.hasTargets()){
            //         hasTarget = true;
            //         target = result.getBestTarget();
            //     }
            // }
            // if (hasTarget){
            //     lostTarget.reset();
            //     targetSpeed = shooter.getCorrectedRPS(calcSpeed());
            //     // var pose = LimelightHelpers.getTargetPose3d_CameraSpace(Constants.LIMELIGHT_NAME);
            //     // if (pose != null){
            //     //     double x = pose.getX(); //Left/right
            //     //     double z = pose.getZ(); //toward target
            //     //     targetSpeed = shooter.getCorrectedRPS(Math.sqrt(x*x + z*z)); //calculates speed required to go into the hub
            //     // }
            // }
            // else {
            //     if (lostTarget.hasElapsed(1)){ //Waits 1 second before exiting the command, this is to stop camera flickers from stopping it
            //         state = State.DONE;
            //         //Add later some sort of warning that april tag not found
            //     }
            // }    
}
