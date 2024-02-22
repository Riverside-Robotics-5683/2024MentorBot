package ravenrobotics.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.VisionConstants;
import ravenrobotics.robot.Constants.VisionConstants.ValidTarget;
public class VisionSubsystem extends SubsystemBase {
    // TODO: Change cameraName to the name of your camera
    private final PhotonCamera camera = new PhotonCamera("photonvision");
        // Query the latest result from PhotonVision
    private PhotonTrackedTarget target;
    private double cameraHeight, cameraPitch;
    private double range, yaw, targetPitch, targetHeight, goalRange;
    private int targetID;
  
    //PID Controllers for Vision Assisted Movement
    //TODO: PID constants should be tuned per robot. Move to constants?
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D); 

    //TODO: Update vision Constants
    public VisionSubsystem(){
        cameraHeight = VisionConstants.kCameraHeight;
        cameraPitch = VisionConstants.kCameraPitchAngle;
     } 
    
    @Override
    public void periodic() { 
        updateTargetData();
     }


    // Get information from target.
    public void updateTargetData(){ // Update the target data in the periodic block
        resetValues(); //reset yaw, targetHeight and target ID to 0
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();// Get the current best target.
            targetID = target.getFiducialId();//The ID of the detected fiducial marker.
            yaw = target.getYaw();//The yaw of the target in degrees (positive right).
                     
            //TODO: print values FOR TESTING
            //System.out.println("Yaw: " + yaw + ", Target ID: " + targetID); 

            //if valid target then set target height, goal range and calculate distance to target (range)
            if (validTarget() != "NONE"){
                
                //check for amp ID and set target height and goal range
                if (validTarget().equals("BLUE_AMP") || validTarget().equals("RED_AMP")){
                    targetHeight = VisionConstants.kTargetHeightAmp;
                    goalRange = VisionConstants.kGoalRangeAmp;
                }
                //check for speaker ID and set target height and goal range
                else if (validTarget().equals("BLUE_SPEAKER") || validTarget().equals("RED_SPEAKER")){
                    targetHeight = VisionConstants.kTargetHeightSpeaker;
                    goalRange = VisionConstants.kGoalRangeSpeaker;
                }

                //Calculate distance to target (range)
                range = PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight, 
                targetHeight,
                cameraPitch,
                Units.degreesToRadians(targetPitch));
            }
            else{
                //error proofing: if goal and range are both 0 VisionCommand should read as completed.
                goalRange = 0;
                range = 0;
            }
            //TODO: print values FOR TESTING
            //System.out.println("Distance to target: " + range + ", Goal Range: " + goalRange);

        }

    }
    //return most recent yaw
    public double getTargetYaw(){
        return yaw;
    }

    //return most recent distance to target
    public double getTargetDistance(){
        return range;
    }
 
    //return target height
    public Double getTargetHeight(){
        return targetHeight;
    }

    //return desired distance from goal
    public Double getGoalRange(){
        return goalRange;
    }

    //reset variable values before updating
    public void resetValues(){
        targetID = 0;
        yaw = 0;
        targetHeight = 0;
    }

    //used to check if target has relevant ID
    public String validTarget(){
        //temp variable
        ValidTarget vt;

        switch (targetID) {
            case VisionConstants.kBlueAmpID:
                vt = ValidTarget.BLUE_AMP;
                break;
            case VisionConstants.kRedAmpID:
                vt = ValidTarget.RED_AMP;
                break;
            case VisionConstants.kBlueCSpeakerID:
                vt = ValidTarget.BLUE_SPEAKER;
                break;
            case VisionConstants.kRedCSpeakerID:
                vt = ValidTarget.RED_SPEAKER;
                break;   
            default:
                vt = ValidTarget.NONE;
                break;
        }
        return vt.toString();
    }

    public String getTColor(){
        String tColor;
        if(validTarget().equals("RED_SPEAKER") || validTarget().equals("RED_AMP")){tColor = "red";}
        else if(validTarget().equals("BLUE_SPEAKER") || validTarget().equals("BLUE_AMP")){tColor = "blue";}
        else{tColor = "none";}
        return tColor;
    }

    //PID Controllers are defined above
    public Double getForwardSpeed(){
        double forwardSpeed;
        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        forwardSpeed = -forwardController.calculate(range, goalRange);
        return forwardSpeed;
    }
        
    public Double getRotationSpeed(){
        double rotationSpeed;
        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -turnController.calculate(yaw, 0);
        return rotationSpeed;
    }

    //TODO: for testing map to button press. Check out how this works.
    /* 
    Request the camera to save a new image file from the output stream with overlays. 
    Images take up space in the filesystem of the PhotonCamera. 
    Calling it frequently will fill up disk space and eventually 
    cause the system to stop working. 
    Clear out images in /opt/photonvision/photonvision_config/imgSaves frequently to prevent issues. 
    */
    public void getCameraImage(){
        // Capture pre-process camera stream image
        // camera.takeInputSnapshot();

        // Capture post-process camera stream image
        camera.takeOutputSnapshot();
    }

     //TODO: Do we want camera vision?
    public void toggleCamera(){
        if(camera.getDriverMode()==false){camera.setDriverMode(true);}
        else{camera.setDriverMode(false);}
    }
    
}