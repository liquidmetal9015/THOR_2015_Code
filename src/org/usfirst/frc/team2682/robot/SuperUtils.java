package org.usfirst.frc.team2682.robot;

public class SuperUtils {
	
	

		
		protected static double limit(double num) {
	        if (num > 1.0) {
	            return 1.0;
	        }
	        if (num < -1.0) {
	            return -1.0;
	        }
	        return num;
	    }
		
	

}
