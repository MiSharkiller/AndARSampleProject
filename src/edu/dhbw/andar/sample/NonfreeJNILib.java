package edu.dhbw.andar.sample;

public class NonfreeJNILib {
	static 
    {
    	try
    	{ 
    		// Load necessary libraries.
    		System.loadLibrary("opencv_java");
    		System.loadLibrary("nonfree");
    		System.loadLibrary("nonfree_jni");
    	}
    	catch( UnsatisfiedLinkError e )
		{
           System.err.println("Native code library failed to load.\n" + e);		
		}
    }

    public static native double[] runDemo(double[] test);
}
