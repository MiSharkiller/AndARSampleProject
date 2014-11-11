package edu.dhbw.andar.sample;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.FloatBuffer;

import javax.microedition.khronos.opengles.GL10;

import android.graphics.Bitmap;
import android.hardware.Camera;


import android.os.AsyncTask;
import android.os.Environment;
import android.util.Log;

import edu.dhbw.andar.ARObject;
import edu.dhbw.andar.ARToolkit;
import edu.dhbw.andar.pub.SimpleBox;
import edu.dhbw.andar.util.GraphicsUtil;


/**
 * An example of an AR object being drawn on a marker.
 * @author tobi
 *
 */
public class CustomObject extends ARObject {
	ARToolkit artoolkit;
	
    
	public CustomObject(String name, String patternName,
			double markerWidth, double[] markerCenter) {
		super(name, patternName, markerWidth, markerCenter);
		float   mat_ambientf[]     = {0f, 1.0f, 0f, 1.0f};
		float   mat_flashf[]       = {0f, 1.0f, 0f, 1.0f};
		float   mat_diffusef[]       = {0f, 1.0f, 0f, 1.0f};
		float   mat_flash_shinyf[] = {50.0f};

		mat_ambient = GraphicsUtil.makeFloatBuffer(mat_ambientf);
		mat_flash = GraphicsUtil.makeFloatBuffer(mat_flashf);
		mat_flash_shiny = GraphicsUtil.makeFloatBuffer(mat_flash_shinyf);
		mat_diffuse = GraphicsUtil.makeFloatBuffer(mat_diffusef);
		
	}
	public CustomObject(String name, String patternName,
			double markerWidth, double[] markerCenter, float[] customColor) {
		super(name, patternName, markerWidth, markerCenter);
		float   mat_flash_shinyf[] = {50.0f};

		mat_ambient = GraphicsUtil.makeFloatBuffer(customColor);
		mat_flash = GraphicsUtil.makeFloatBuffer(customColor);
		mat_flash_shiny = GraphicsUtil.makeFloatBuffer(mat_flash_shinyf);
		mat_diffuse = GraphicsUtil.makeFloatBuffer(customColor);
		
	}
	
	/**
	 * Just a box, imported from the AndAR project.
	 */
	private SimpleBox box = new SimpleBox();
	private FloatBuffer mat_flash;
	private FloatBuffer mat_ambient;
	private FloatBuffer mat_flash_shiny;
	private FloatBuffer mat_diffuse;
	
	/**
	 * Everything drawn here will be drawn directly onto the marker,
	 * as the corresponding translation matrix will already be applied.
	 */
	@Override
	public final void draw(GL10 gl) {
		super.draw(gl);
		
		gl.glMaterialfv(GL10.GL_FRONT_AND_BACK, GL10.GL_SPECULAR,mat_flash);
		gl.glMaterialfv(GL10.GL_FRONT_AND_BACK, GL10.GL_SHININESS, mat_flash_shiny);	
		gl.glMaterialfv(GL10.GL_FRONT_AND_BACK, GL10.GL_DIFFUSE, mat_diffuse);	
		gl.glMaterialfv(GL10.GL_FRONT_AND_BACK, GL10.GL_AMBIENT, mat_ambient);

	    //draw cube
	    gl.glColor4f(0, 1.0f, 0, 1.0f);
	    gl.glTranslatef( 0.0f, 0.0f, 12.5f );
	    
	    //draw the box
	    box.draw(gl);
	    Log.d("Draw","box");
	    
	}
	@Override
	public void init(GL10 gl) {
		
	}
	public float[] getCameraMatrix(){
		return this.glCameraMatrix;
	}
	
	
	
	private class LoadingDataAsyncTask extends AsyncTask<String, Integer, Integer>{

		@Override
		protected Integer doInBackground(String... arg0) {
			// TODO Auto-generated method stub
			try {
				Thread.sleep(7000);
				Log.d("hey", "ready?");
				//getData();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			return null;
		}
	
	}
}
