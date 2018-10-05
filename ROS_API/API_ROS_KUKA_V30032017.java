package application;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.ErrorHandlingAction;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IErrorHandler;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.OrientationReferenceSystem;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;


/*******************************************************
 * Implementation of API for KUKA LBR 7/14
 * Sheffield Robotics Laboratory
 * http://www.sheffieldrobotics.ac.uk/
 * The University of Sheffield
 * 
 * Author: Saeid Mokaram
 * Email: s.mokaram@sheffield.ac.uk , saeid.mokaram@gmail.com
 * Date: 30/3/2017
*******************************************************/

/*******************************************************
* You need to make a tool1 in 'Template data->Template Group of Tools' and name it 'tool'.
* Insert new empty frame under 'tool1' and name it 'TCP'.
* In properties of the 'tool1', set the transformation values.
* In properties of the 'tool1', select the 'TCP' as 'Default Motion Frame'.
* Synchronize the robot with Sunrise Workbench. 
* On the robot control pad, perform Automatic load data determination for the tool.
*******************************************************/

public class API_ROS_KUKA_V30032017 extends RoboticsAPIApplication {
	public boolean RUN = false;  // Is socket Connected and app running?
	public boolean isCompliance = false;
	public boolean isCollision = false;
	
	public LBR lbr;  // Kuka iiwa robot
	public Tool tool;
	
	public PrintWriter outputStream;
	public BufferedReader inputStream;
    public Socket skt;  // TCP Socket 
    public String CommandStr; // Command String
    String []strParams = new String[20];
    float []params = new float[10];
    
    public long LastReceivedTime = System.currentTimeMillis();
	public double JointAcceleration;
	public double JointVelocity;
	public double JointJerk;
	
	public double Acceleration = 1.0;
	public double Velocity = 1.0;
	
	public double CartVelocity = 10;

	public IMotionContainer handleCompliance;
	
	public CartesianImpedanceControlMode cartImpCtrlMode = new CartesianImpedanceControlMode();
	public boolean isCartImpCtrlMode = false;

	private IErrorHandler errorHandler;
	//===========================================================
	
	public void initialize() {
		getController("KUKA_Sunrise_Cabinet_1");
		lbr = getContext().getDeviceFromType(LBR.class);
		tool = getApplicationData().createFromTemplate("tool1");		
		tool.attachTo(lbr.getFlange());
		
		JointAcceleration  = 1.0;
		JointVelocity = 1.0;
		JointJerk = 1.0;
		
		cartImpCtrlMode.parametrize(CartDOF.X).setStiffness(5000.0);
		cartImpCtrlMode.parametrize(CartDOF.Y).setStiffness(5000.0);
		cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(5000.0);
		cartImpCtrlMode.parametrize(CartDOF.A).setStiffness(300.0);
		cartImpCtrlMode.parametrize(CartDOF.B).setStiffness(300.0);
		cartImpCtrlMode.parametrize(CartDOF.C).setStiffness(300.0);
		cartImpCtrlMode.parametrize(CartDOF.ALL).setDamping(1.0);

		errorHandler = new IErrorHandler() {
			@Override
			public ErrorHandlingAction handleError(Device device,
					IMotionContainer failedContainer,
					List<IMotionContainer> canceledContainers) {
                            getLogger().warn("Excecution of the following motion failed: "
                                             + failedContainer.getCommand().toString());
                            getLogger().info("The following motions will not be executed:");
                            for (int i = 0; i < canceledContainers.size(); i++) {
                                getLogger().info(canceledContainers.get(i)
                                                 .getCommand().toString());
                            }
                            return ErrorHandlingAction.Ignore;
			}
                    };
		getApplicationControl()
                    .registerMoveAsyncErrorHandler(errorHandler);
	}

	public void socketConnection()  // Connecting to server at 172.31.1.50 Port:1234
	{
		System.out.println("Connecting to server at 172.31.1.50 Port:1234");
		
		while (true){
			try{
			    skt = new Socket("172.31.1.50", 1234); // Modify the IP and port depending on the system which is running the ROS-KUKA node server if it is required.
		    	System.out.println("KUKA iiwa is connected to the server.");
		    	break;
			}
			catch(IOException e1){
		        System.out.println("ERROR connecting to the server!");	        
			}
		}

	    try{
	    	outputStream = new PrintWriter(skt.getOutputStream(), true);
	    	inputStream = new BufferedReader(new InputStreamReader(skt.getInputStream()));
	    	RUN = true;
	    }
	    catch(IOException e)
	    {
	    	System.out.println("Error creating inPort and outPort");
	    }
	}
	
	public void reply(PrintWriter outputStream, String buffer)
	{
        outputStream.write(buffer);
        outputStream.flush();	
	}

	public String getLine(BufferedReader inputStream)
	{
		String line;
		try{
			
			while(!inputStream.ready()){}
				 			
			line = inputStream.readLine();
	    	//System.out.println("Command received: " + line);
	    	
	    	return line;    	
		}
		catch(Exception e){		
			return "Error command";
		}
	}
	
	public void behaviourAfterCollision() { 
		isCollision = true;
		IMotionContainer handle;
		
		CartesianImpedanceControlMode CollisionSoft = new CartesianImpedanceControlMode();
		CollisionSoft.parametrize(CartDOF.ALL).setDamping(.7);
		CollisionSoft.parametrize(CartDOF.ROT).setStiffness(100);
		CollisionSoft.parametrize(CartDOF.TRANSL).setStiffness(600);
				
		handle = tool.moveAsync(positionHold(CollisionSoft, -1, TimeUnit.SECONDS));
		/*getApplicationUI().displayModalDialog(ApplicationDialogType.WARNING,
			"Collision Has Occurred!\n\n LBR is compliant...",
			"Continue");*/
		handle.cancel();
	}

    public ICondition defineSensitivity() {
		//double sensCLS = getApplicationData().getProcessData("sensCLS").getValue(); // Uncomment if you have "sensCLS" defined.
		double sensCLS = 30; // Modify the value if required.
		
		//Offsetkompensation
		double actTJ1 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J1);
		double actTJ2 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J2);
		double actTJ3 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J3);
		double actTJ4 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J4);
		double actTJ5 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J5);
		double actTJ6 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J6);
		double actTJ7 = lbr.getExternalTorque().getSingleTorqueValue(JointEnum.J7);
		
		
		//Abbruchbedingungen pro Achse
		JointTorqueCondition jt1 = new JointTorqueCondition(JointEnum.J1, -sensCLS+actTJ1, sensCLS+actTJ1);
		JointTorqueCondition jt2 = new JointTorqueCondition(JointEnum.J2, -sensCLS+actTJ2, sensCLS+actTJ2);
		JointTorqueCondition jt3 = new JointTorqueCondition(JointEnum.J3, -sensCLS+actTJ3, sensCLS+actTJ3);
		JointTorqueCondition jt4 = new JointTorqueCondition(JointEnum.J4, -sensCLS+actTJ4, sensCLS+actTJ4);
		JointTorqueCondition jt5 = new JointTorqueCondition(JointEnum.J5, -sensCLS+actTJ5, sensCLS+actTJ5);
		JointTorqueCondition jt6 = new JointTorqueCondition(JointEnum.J6, -sensCLS+actTJ6, sensCLS+actTJ6);
		JointTorqueCondition jt7 = new JointTorqueCondition(JointEnum.J7, -sensCLS+actTJ7, sensCLS+actTJ7);

		ICondition forceCon = jt1.or(jt2, jt3, jt4, jt5, jt6, jt7);
		return forceCon;
	}
	
    private IMotionContainer _currentMotion;
    public void MoveSafe(MotionBatch MB) {
		ICondition forceCon = defineSensitivity();
		
		ICallbackAction ica = new ICallbackAction() {
			@Override
			public void onTriggerFired(IFiredTriggerInfo triggerInformation)
			{
				triggerInformation.getMotionContainer().cancel();
				behaviourAfterCollision();
			}
		};

		if(isCartImpCtrlMode)
		{
			MB.setJointAccelerationRel(JointAcceleration)
			.setJointVelocityRel(JointVelocity)
			.setMode(cartImpCtrlMode)
			.triggerWhen(forceCon, ica);
		}
		else
		{
			MB.setJointAccelerationRel(JointAcceleration)
			.setJointVelocityRel(JointVelocity)
			.triggerWhen(forceCon, ica);
		}
		
		
		if(lbr.isReadyToMove())
			this._currentMotion=tool.moveAsync(MB);
	}

	//===========================================================
    public void setTool(String message){  // receives: "setTool TOOLNAME"
    	strParams = message.split(" ");
		if (strParams.length==2){
			tool = getApplicationData().createFromTemplate(strParams[1]);		
			tool.attachTo(lbr.getFlange());
			getLogger().info("Switched to " + strParams[1]);
		}
		else{
			getLogger().info("Unacceptable 'setTool' command!");
		}
			
	}
    
	public void setJointAcceleration(String message){
		strParams = message.split(" ");
		if (strParams.length==2)
			params[0] = Float.parseFloat(strParams[1]);
		if (0.0 < params[0] && params[0] <= 1.0)
			JointAcceleration = params[0];
		else
			getLogger().info("JointAcceleration must be 0<JA<=1");
	}
	
	public void setJointVelocity(String message){
		strParams = message.split(" ");
		if (strParams.length==2)
			params[0] = Float.parseFloat(strParams[1]);
		if (0.0 < params[0] && params[0] <= 1.0)
			JointVelocity = params[0];
		else
			getLogger().info("JointVelocity must be 0<JV<=1");
	}
	
	public void setJointJerk(String message){
		strParams = message.split(" ");
		if (strParams.length==2)
			params[0] = Float.parseFloat(strParams[1]);
		if (0.0 < params[0] && params[0] <= 1.0)
			JointJerk = params[0];
		else
			getLogger().info("JointJerk must be 0<JJ<=1");
	}
	
	public void CartVelocity(String message){
		strParams = message.split(" ");
		if (strParams.length==2)
			params[0] = Float.parseFloat(strParams[1]);
		if (0.0 < params[0] && params[0] <= 10000)
			CartVelocity = params[0];
		else
			getLogger().info("CartVelocity must be 0<CV<=10000");
	}
	
 	public void setPosition(String message) // "setPosition A0 A1 - A3 A4 A5 A6" It's a ptp motion only.  
	{	strParams = message.split(" ");
		
		double A0 = lbr.getCurrentJointPosition().get(0);
		double A1 = lbr.getCurrentJointPosition().get(1);
		double A2 = lbr.getCurrentJointPosition().get(2);
		double A3 = lbr.getCurrentJointPosition().get(3);
		double A4 = lbr.getCurrentJointPosition().get(4);
		double A5 = lbr.getCurrentJointPosition().get(5);
		double A6 = lbr.getCurrentJointPosition().get(6);
		
		if (strParams.length==8){
			if( !strParams[1].equals("-") )
				A0 = Math.toRadians( Float.parseFloat(strParams[1]) );
			if( !strParams[2].equals("-") )
				A1 = Math.toRadians( Float.parseFloat(strParams[2]) );
			if( !strParams[3].equals("-") )
				A2 = Math.toRadians( Float.parseFloat(strParams[3]) );
			if( !strParams[4].equals("-") )
				A3 = Math.toRadians( Float.parseFloat(strParams[4]) );
			if( !strParams[5].equals("-") )
				A4 = Math.toRadians( Float.parseFloat(strParams[5]) );
			if( !strParams[6].equals("-") )
				A5 = Math.toRadians( Float.parseFloat(strParams[6]) );
			if( !strParams[7].equals("-") )
				A6 = Math.toRadians( Float.parseFloat(strParams[7]) );		
		
		 	MotionBatch motion = new MotionBatch(ptp(A0,A1,A2,A3,A4,A5,A6).setJointJerkRel(JointJerk));
		 	MoveSafe(motion);
		}
		else{
			getLogger().info("Unacceptable 'setPosition' command!");
		}
	}
	
 	public void setPositionXYZABC(String message) // setPositionXYZABC x y z a - c ptp/lin
    {
 		double x = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getX();
        double y = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getY();
        double z = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getZ();
        double a = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getAlphaRad();
        double b = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getBetaRad();
        double c = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getGammaRad();
        
        strParams = message.split(" ");
        if (strParams.length==8){

        	if( !strParams[1].equals("-"))
        		x = Float.parseFloat(strParams[1]);
        	if( !strParams[2].equals("-"))
        		y = Float.parseFloat(strParams[2]);
        	if( !strParams[3].equals("-"))
        		z = Float.parseFloat(strParams[3]);
        	if( !strParams[4].equals("-"))
        		a = Math.toRadians( Float.parseFloat(strParams[4]) );
        	if( !strParams[5].equals("-"))
        		b = Math.toRadians( Float.parseFloat(strParams[5]) );
        	if( !strParams[6].equals("-"))
        		c = Math.toRadians( Float.parseFloat(strParams[6]) );
            
        	Frame f = new Frame(getApplicationData().getFrame("/Air"), x, y, z, a, b, c );
        	if( strParams[7].equals("ptp"))
        		MoveSafe( new MotionBatch(ptp(f).setJointJerkRel(JointJerk) ));
        	else if( strParams[7].equals("lin"))
        		MoveSafe( new MotionBatch(lin(f).setJointJerkRel(JointJerk).setCartVelocity(CartVelocity) ));
        	else
        		getLogger().info("Unacceptable motion! ptp/lin ?");
        }
        else{
            getLogger().info("Unacceptable setPositionXYZABC command!");
        }
    }
	
	public void MoveXYZABC(String message) // "MoveXYZABC x y 0 a 0 c"  It's a linRel motion only.
    {
		strParams = message.split(" ");
        if (strParams.length==7){

            double x = Float.parseFloat(strParams[1]);
            double y = Float.parseFloat(strParams[2]);
            double z = Float.parseFloat(strParams[3]);
            double a = Math.toRadians( Float.parseFloat(strParams[4]) );
            double b = Math.toRadians( Float.parseFloat(strParams[5]) );
            double c = Math.toRadians( Float.parseFloat(strParams[6]) );
            
            MotionBatch motion = new MotionBatch(linRel(x, y, z, a, b, c).setJointJerkRel(JointJerk).setCartVelocity(CartVelocity));
            MoveSafe(motion);
        }
        else{
        	getLogger().info("Unacceptable MoveXYZABC command!");
        }
    }
	
	public void MoveCirc(String message)  // "MoveCirc x1 y1 z1 a1 b1 c1 x2 y2 z2 a2 b2 c2 blending"
    {   
		strParams = message.split(" ");
        if (strParams.length==14){
        	double x1 = Float.parseFloat(strParams[1]);
            double y1 = Float.parseFloat(strParams[2]);
            double z1 = Float.parseFloat(strParams[3]);
            double a1 = Math.toRadians( Float.parseFloat(strParams[4]) );
            double b1 = Math.toRadians( Float.parseFloat(strParams[5]) );
            double c1 = Math.toRadians( Float.parseFloat(strParams[6]) );
            
            double x2 = Float.parseFloat(strParams[7]);
            double y2 = Float.parseFloat(strParams[8]);
            double z2 = Float.parseFloat(strParams[9]);
            double a2 = Math.toRadians( Float.parseFloat(strParams[10]) );
            double b2 = Math.toRadians( Float.parseFloat(strParams[11]) );
            double c2 = Math.toRadians( Float.parseFloat(strParams[12]) );
            
            double BlendingOri = Math.toRadians( Float.parseFloat(strParams[13]) );
            
            Frame f1 = new Frame(getApplicationData().getFrame("/Air"), x1, y1, z1, a1, b1, c1 );
            Frame f2 = new Frame(getApplicationData().getFrame("/Air"), x2, y2, z2, a2, b2, c2 );
            MoveSafe( new MotionBatch( circ(f1, f2).setBlendingOri(BlendingOri).setOrientationReferenceSystem(OrientationReferenceSystem.Path).setCartVelocity(CartVelocity) ));
            
        }
        else{
            getLogger().info("Unacceptable MoveCirc command!");
        }
    }
	
	public void setForceStop(){
		try{
			if(!(this._currentMotion.isFinished() ||
					this._currentMotion.hasError()))
				this._currentMotion.cancel();
		}
		catch(Exception e){
			// nop
		};
			
		CartesianImpedanceControlMode ForceStop = new CartesianImpedanceControlMode();
		ForceStop.parametrize(CartDOF.ALL).setDamping(.7);
		ForceStop.parametrize(CartDOF.ROT).setStiffness(300);
		ForceStop.parametrize(CartDOF.TRANSL).setStiffness(5000);
		
		handleCompliance = lbr.moveAsync(positionHold(ForceStop, 0, TimeUnit.SECONDS));
		handleCompliance.cancel();
		isCompliance = false;
		
		behaviourAfterCollision();
		
		getLogger().info("ForceStop!");
	}
	
	public void setCompliance(String message){   // "setCompliance 10 10 5000 300 300 300"
		// setCompliance "X-Stiffness" "Y-Stiffness" "Z-Stiffness" "A-R-Stiffness" "b-R-Stiffness" "c-R-Stiffness"
		// x,y = 10 not to soft (robot stands by its own)
		// x = 5000 max Stiffness.
		// az, by, cx = 300 max rotational Stiffness.
		
		CartesianImpedanceControlMode soft = new CartesianImpedanceControlMode();
		strParams = message.split(" ");
		if (strParams.length==7){
			float x = Float.parseFloat(strParams[1]);
			float y = Float.parseFloat(strParams[2]);
			float z = Float.parseFloat(strParams[3]);
			float az = Float.parseFloat(strParams[4]);
			float by = Float.parseFloat(strParams[5]);
			float cx = Float.parseFloat(strParams[6]);
			
			soft.parametrize(CartDOF.ALL).setDamping(0.7);
			soft.parametrize(CartDOF.X).setStiffness(x);
			soft.parametrize(CartDOF.Y).setStiffness(y);
			soft.parametrize(CartDOF.Z).setStiffness(z);
			soft.parametrize(CartDOF.A).setStiffness(az);
			soft.parametrize(CartDOF.B).setStiffness(by);
			soft.parametrize(CartDOF.C).setStiffness(cx);
			
			if (isCompliance)
				handleCompliance.cancel();
			
			handleCompliance = tool.moveAsync(positionHold(soft, -1, TimeUnit.SECONDS));
			isCompliance = true;
			getLogger().info("Compliance is ON");	
		}
		else{
			getLogger().info("Unacceptable setCompliance command!");
		}
	}
	
	public void setCartImpCtrl(String message){  // "setCartImpCtrl 5000 5000 500 300 300 300 Damping"
		strParams = message.split(" ");
		if (strParams.length==8){
			float x = Float.parseFloat(strParams[1]);
			float y = Float.parseFloat(strParams[2]);
			float z = Float.parseFloat(strParams[3]);
			float az = Float.parseFloat(strParams[4]);
			float by = Float.parseFloat(strParams[5]);
			float cx = Float.parseFloat(strParams[6]);
			float Damping = Float.parseFloat(strParams[7]);
			
			cartImpCtrlMode.parametrize(CartDOF.X).setStiffness(x);
			cartImpCtrlMode.parametrize(CartDOF.Y).setStiffness(y);
			cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(z);
			cartImpCtrlMode.parametrize(CartDOF.A).setStiffness(az);
			cartImpCtrlMode.parametrize(CartDOF.B).setStiffness(by);
			cartImpCtrlMode.parametrize(CartDOF.C).setStiffness(cx);
			
			if (0.0 < Damping && Damping <= 1.0)
				cartImpCtrlMode.parametrize(CartDOF.ALL).setDamping(Damping);
			else{
				cartImpCtrlMode.parametrize(CartDOF.ALL).setDamping(1.0);
				getLogger().info("CartImpCtrl: Damping must be 0<JA<=1. Damping set to 1.0");
			}
			isCartImpCtrlMode = true;
			getLogger().info("New CartImpCtrl mode is set.");
		}
		else{
			getLogger().info("Unacceptable CartImpCtrl command!");
		}
	}
	
	public void resetCartImpCtrl(){
		cartImpCtrlMode.parametrize(CartDOF.X).setStiffness(5000);
		cartImpCtrlMode.parametrize(CartDOF.Y).setStiffness(5000);
		cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(5000);
		cartImpCtrlMode.parametrize(CartDOF.ROT).setStiffness(300);
		cartImpCtrlMode.parametrize(CartDOF.ALL).setDamping(1.0);
		
		isCartImpCtrlMode = false;
		getLogger().info("CartImpCtrl is OFF!");
	}
	
	public void resetCompliance(){
		CartesianImpedanceControlMode resetComp = new CartesianImpedanceControlMode();
		resetComp.parametrize(CartDOF.ALL).setDamping(.7);
		resetComp.parametrize(CartDOF.ROT).setStiffness(300);
		resetComp.parametrize(CartDOF.TRANSL).setStiffness(5000);
		
		if (isCompliance){
			handleCompliance.cancel();
			handleCompliance = tool.moveAsync(positionHold(resetComp, 0, TimeUnit.SECONDS));
			isCompliance = false;
		}
		
		getLogger().info("Compliance is OFF");
	}
	
	public void getJointPosition(PrintWriter outputStream)
	{
		LastReceivedTime = System.currentTimeMillis();
		double a0 = Math.toDegrees( lbr.getCurrentJointPosition().get(0) );
		double a1 = Math.toDegrees( lbr.getCurrentJointPosition().get(1) );
		double a2 = Math.toDegrees( lbr.getCurrentJointPosition().get(2) );
		double a3 = Math.toDegrees( lbr.getCurrentJointPosition().get(3) );
		double a4 = Math.toDegrees( lbr.getCurrentJointPosition().get(4) );
		double a5 = Math.toDegrees( lbr.getCurrentJointPosition().get(5) );
		double a6 = Math.toDegrees( lbr.getCurrentJointPosition().get(6) );
		
		String buffer = Double.toString(a0) + ", " + Double.toString(a1) + ", " 
				      + Double.toString(a2) + ", " + Double.toString(a3) + ", " 
				      + Double.toString(a4) + ", " + Double.toString(a5) + ", "
					  + Double.toString(a6);
		
        reply(outputStream, ">"+"Joint_Pos " + buffer);
	}
	
	public void getToolPosition(PrintWriter outputStream)
	{
		double x = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getX();
		double y = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getY();
		double z = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getZ();
		
		double a = Math.toDegrees( lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getAlphaRad() );
		double b = Math.toDegrees( lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getBetaRad() );
		double g = Math.toDegrees( lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getGammaRad() );
		
        String buffer = Double.toString(x) + ", " + Double.toString(y) + ", " + Double.toString(z) + ", " + Double.toString(a) + ", " + Double.toString(b) + ", " + Double.toString(g);
		
        LastReceivedTime = System.currentTimeMillis();
		
        reply(outputStream, ">"+"Tool_Pos " + buffer);
	}
	
	public void getIsCompliance(PrintWriter outputStream)
	{
		LastReceivedTime = System.currentTimeMillis();
		
		if (isCompliance)
			reply(outputStream, ">"+"isCompliance " + "true");
		else
			reply(outputStream, ">"+"isCompliance " + "false");
	}
	
	public void getToolForce(PrintWriter outputStream)
	{
		double x = lbr.getExternalForceTorque(tool.getDefaultMotionFrame()).getForce().getX();
		double y = lbr.getExternalForceTorque(tool.getDefaultMotionFrame()).getForce().getY();
		double z = lbr.getExternalForceTorque(tool.getDefaultMotionFrame()).getForce().getZ();
		
        String buffer = Double.toString(x) + ", " + Double.toString(y) + ", " + Double.toString(z);
		
        LastReceivedTime = System.currentTimeMillis();
		
        reply(outputStream, ">"+"Tool_Force " + buffer);
	}
	
	public void getToolTorque(PrintWriter outputStream)
	{
		double x = lbr.getExternalForceTorque(tool.getDefaultMotionFrame()).getTorque().getX();
		double y = lbr.getExternalForceTorque(tool.getDefaultMotionFrame()).getTorque().getY();
		double z = lbr.getExternalForceTorque(tool.getDefaultMotionFrame()).getTorque().getZ();
		
        String buffer = Double.toString(x) + ", " + Double.toString(y) + ", " + Double.toString(z);
		
        LastReceivedTime = System.currentTimeMillis();
		
        reply(outputStream, ">"+"Tool_Torque " + buffer);
	}
		
    public void getJointAcceleration(PrintWriter outputStream)
	{
		LastReceivedTime = System.currentTimeMillis();
        reply(outputStream, ">"+"JointAcceleration " + Double.toString(JointAcceleration));
	}
	
	public void getJointVelocity(PrintWriter outputStream)
	{
		LastReceivedTime = System.currentTimeMillis();
        reply(outputStream, ">"+"JointVelocity " + Double.toString(JointVelocity));
	}
	
	public void getJointJerk(PrintWriter outputStream)
	{
		LastReceivedTime = System.currentTimeMillis();
        reply(outputStream, ">"+"JointJerk " + Double.toString(JointJerk));
	}
	
	public void getIsCollision(PrintWriter outputStream)
	{
		LastReceivedTime = System.currentTimeMillis();
        if (isCollision)
        	reply(outputStream, ">"+"isCollision " + "true");
        else
        	reply(outputStream, ">"+"isCollision " + "false");
	}
	
	public void getIsReadyToMove(PrintWriter outputStream)
	{
		LastReceivedTime = System.currentTimeMillis();
        if (lbr.isReadyToMove())
        	reply(outputStream, ">"+"isReadyToMove " + "true");
        else
        	reply(outputStream, ">"+"isReadyToMove " + "false");
	}
	
	public void getIsMastered(PrintWriter outputStream)
	{
		LastReceivedTime = System.currentTimeMillis();
        if (lbr.isMastered())
        	reply(outputStream, ">"+"isMastered " + "true");
        else
        	reply(outputStream, ">"+"isMastered " + "false");
	}

	private void JointOutOfRange() { 
		CartesianImpedanceControlMode OutOfRange = new CartesianImpedanceControlMode();
		OutOfRange.parametrize(CartDOF.ALL).setDamping(.7);
		OutOfRange.parametrize(CartDOF.ROT).setStiffness(300);
		OutOfRange.parametrize(CartDOF.TRANSL).setStiffness(5000);
		
		IMotionContainer handle;
		handle = tool.moveAsync(positionHold(OutOfRange, -1, TimeUnit.SECONDS));
		getApplicationUI().displayModalDialog(ApplicationDialogType.WARNING,
				"Joint Out Of Range!\n\n",
				"Continue");
		handle.cancel();
	}
	
	private void getOperationMode(PrintWriter outputStream) {
		String opMode = lbr.getOperationMode().toString();
		reply(outputStream, ">"+"OperationMode " + opMode);
	}
	
	public void sleep(String message){  // sleep in seconds.
		strParams = message.split(" ");
		float delay;
		
		if (strParams.length==2)
		{
			delay = Float.parseFloat(strParams[1]);
			if(delay >= 0)
				ThreadUtil.milliSleep((long) (delay*1000));
			else
				getLogger().info("Delay cannot be a negative value!");
		}
		else
			getLogger().info("Unacceptable sleep command!");
	}

	public void getIsFinished(PrintWriter outputStream)
	{
		LastReceivedTime = System.currentTimeMillis();
		try{
			if(this._currentMotion.isFinished())
	        	reply(outputStream, ">"+"isFinished " + "true");
			else
	        	reply(outputStream, ">"+"isFinished " + "false");
		}
		catch(Exception e){
        	reply(outputStream, ">"+"isFinished " + "true");
		};
	}
	public void getHasError(PrintWriter outputStream)
	{
		LastReceivedTime = System.currentTimeMillis();
		try{
                    if(this._currentMotion.hasError())
                        reply(outputStream, ">"+"hasError " + "true");
                    else
                        reply(outputStream, ">"+"hasError " + "false");
		}
		catch(Exception e){
                    reply(outputStream, ">"+"hasError " + "true");
		};
	}
	//===========================================================
	
	public Thread Send_iiwa_data = new Thread(){
	    public void run(){
	    	while (RUN)
	    	{
	    		getJointPosition(outputStream);
	    		getToolPosition(outputStream);
	    		getToolForce(outputStream);
	    		getToolTorque(outputStream);
	    		getIsCompliance(outputStream);
	    		getJointAcceleration(outputStream);
	    		getJointVelocity(outputStream);
	    		getJointJerk(outputStream);
	    		getIsCollision(outputStream);
	    		getIsReadyToMove(outputStream);
	    		getIsMastered(outputStream);
	    		getOperationMode(outputStream);
	    		getIsFinished(outputStream);
                        getHasError(outputStream);

	    		ThreadUtil.milliSleep(100);
	    	}
	    }
	};

	public Thread MonitorWorkspace = new Thread(){
		JointPosition pos_last;
		JointPosition pos_tmp;
		double x;
		double y;
		double z;
		
	    public void run(){
	    	while (RUN)
	    	{	
	    		//============================================
	    		pos_tmp = lbr.getCurrentJointPosition();

	    		if ( (isCompliance) &&
	    			(Math.toDegrees(pos_tmp.get(0)) < -165 || Math.toDegrees(pos_tmp.get(0)) > 165 ||
	    		     Math.toDegrees(pos_tmp.get(1)) < -115 || Math.toDegrees(pos_tmp.get(1)) > 115 ||
	    		     Math.toDegrees(pos_tmp.get(2)) < -165 || Math.toDegrees(pos_tmp.get(2)) > 165 ||
	    		     Math.toDegrees(pos_tmp.get(3)) < -115 || Math.toDegrees(pos_tmp.get(3)) > 115 ||
	    		     Math.toDegrees(pos_tmp.get(4)) < -165 || Math.toDegrees(pos_tmp.get(4)) > 165 ||
	    		     Math.toDegrees(pos_tmp.get(5)) < -115 || Math.toDegrees(pos_tmp.get(5)) > 115 ||
	    		     Math.toDegrees(pos_tmp.get(6)) < -170 || Math.toDegrees(pos_tmp.get(6)) > 170 ) )
	    		{
	    				setForceStop();
	    			 	MoveSafe(new MotionBatch(ptp(pos_last).setJointJerkRel(JointJerk)));
	    			 	JointOutOfRange();
	    		}
	    		else
	    			pos_last = pos_tmp;
	    		//============================================
	    		
	    		x = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getX();
	    		y = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getY();
	    		z = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame()).getZ();
	    		
	    		/*
	    		
	    		if isInGreen(x,y,z): //Green zone
	    		{
	    			JointVelocity = 1;
	    			JointAcceleration = 1;
	    		}
	    		else if isInYello(x,y,z): //Yello zone
	    		{
	    			JointVelocity = 1;
	    			JointAcceleration = 1;
	    		}
	    		else if isInRead(x,y,z) //Red zone
	    		{
	    			JointVelocity = 1;
    				JointAcceleration = 1;
	    		}
	    		
	    		*/
	    		//============================================
	    		
	    		ThreadUtil.milliSleep(10);
	    	}
	    }
	};
	
	//===========================================================

	public void run() {

		socketConnection();
		Send_iiwa_data.start();
		MonitorWorkspace.start();
		
		while( RUN )
		{
	    	CommandStr = getLine(inputStream);
	    	String []lineSplt = CommandStr.split(" ");

			if( (lineSplt[0].toString()).equals("setPosition".toString()))
				setPosition(CommandStr);
			
			if( (lineSplt[0].toString()).equals("setPositionXYZABC".toString()))
				setPositionXYZABC(CommandStr);
			
			if( (lineSplt[0].toString()).equals("MoveXYZABC".toString()))
				MoveXYZABC(CommandStr);
			
			if( (lineSplt[0].toString()).equals("setCompliance".toString()))
				setCompliance(CommandStr);
			
			if( (lineSplt[0].toString()).equals("resetCartImpCtrl".toString()))
				resetCartImpCtrl();
			
			if( (lineSplt[0].toString()).equals("resetCompliance".toString()))
				resetCompliance();
			
			if( (lineSplt[0].toString()).equals("setCartImpCtrl".toString()))
				setCartImpCtrl(CommandStr);
			
			if( (lineSplt[0].toString()).equals("setTool".toString()))
				setTool(CommandStr);
			
			if( (lineSplt[0].toString()).equals("setJointAcceleration".toString()))
				setJointAcceleration(CommandStr);
			
			if( (lineSplt[0].toString()).equals("setJointVelocity".toString()))
				setJointVelocity(CommandStr);
			
			if( (lineSplt[0].toString()).equals("setJointJerk".toString()))
				setJointJerk(CommandStr);
			
			if( (lineSplt[0].toString()).equals("setCartVelocity".toString()))
				CartVelocity(CommandStr);
			
			if( (lineSplt[0].toString()).equals("resetCollision".toString()))
				isCollision = false;
			
			if( (lineSplt[0].toString()).equals("forceStop".toString()))
				setForceStop();
			
			if( (lineSplt[0].toString()).equals("MoveCirc".toString()))
                MoveCirc(CommandStr);
			
			if( (lineSplt[0].toString()).equals("sleep".toString()))
				sleep(CommandStr);
			
			if( skt.isInputShutdown() || !skt.isConnected() || skt.isOutputShutdown() || skt.isClosed())
			{
				try {
					skt.close();
				} catch (IOException e) {
					System.out.println("ERROR closing the port!");
				}
				RUN = false;
			}
		}
		
		System.out.println("- - - APPLICATION TERMINATED - - -");
	}

	//===========================================================
	
	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args){
		API_ROS_KUKA_V30032017 app = new API_ROS_KUKA_V30032017();
		app.runApplication();
		
	}
}

