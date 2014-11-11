package edu.dhbw.andar.sample;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import android.graphics.Bitmap;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceHolder.Callback;
import edu.dhbw.andar.ARObject;
import edu.dhbw.andar.ARToolkit;
import edu.dhbw.andar.AndARActivity;
import edu.dhbw.andar.CameraPreviewHandler;
import edu.dhbw.andar.exceptions.AndARException;
import edu.dhbw.andar.interfaces.MarkerVisibilityListener;
/**
 * Example of an application that makes use of the AndAR toolkit.
 * @author Tobi
 *
 */
public class CustomActivity extends AndARActivity{

	CustomObject someObject;
	ARToolkit artoolkit;
	CameraPreviewHandler  camH;
	float[] glCameraMatrix;
	
	CameraPreviewer cp = new CameraPreviewer();
	
	@Override
	public void onCreate(Bundle savedInstanceState) {
		
		super.onCreate(savedInstanceState);
		CustomRenderer renderer = new CustomRenderer();//optional, may be set to null
		super.setNonARRenderer(renderer);//or might be omited
		try {
			//register a object for each marker type
			
			artoolkit = super.getArtoolkit();
			someObject = new CustomObject
				("test", "patt.hiro", 80.0, new double[]{0,0});
			artoolkit.registerARObject(someObject);
			someObject = new CustomObject
			("test", "android.patt", 80.0, new double[]{0,0});
			artoolkit.registerARObject(someObject);
			someObject = new CustomObject
			("test", "barcode.patt", 80.0, new double[]{0,0});
			artoolkit.registerARObject(someObject);
		} catch (AndARException ex){
			//handle the exception, that means: show the user what happened
			
		}	
		
		
		//this.takeScreenshot();
		
		
		
		
		
		new LoadingDataAsyncTask().execute("");
		
		float[] ha = someObject.getCameraMatrix();
		double[] yo = someObject.getTransMatrix();
		//double[] yes = NonfreeJNILib.runDemo();
		double [] hey = {3.3, 2.2, 1.1};
		//NonfreeJNILib.runDemo(hey);
		
		
		//Log.d("YES",String.valueOf(yes[2]));
		
		//Log.d("test123",String.valueOf(ha));
		/*Log.d("test321",String.valueOf(ha[0]));
		Log.d("test321",String.valueOf(ha[1]));
		Log.d("test321",String.valueOf(ha[2]));
		Log.d("test321",String.valueOf(ha[3]));
		Log.d("test321",String.valueOf(ha[4]));
		Log.d("test321",String.valueOf(ha[5]));
		Log.d("test321",String.valueOf(ha[6]));
		Log.d("test321",String.valueOf(ha[7]));
		Log.d("test321",String.valueOf(ha[8]));
		Log.d("test321",String.valueOf(ha[9]));
		Log.d("test321",String.valueOf(ha[10]));
		Log.d("test321",String.valueOf(ha[11]));
		Log.d("test321",String.valueOf(ha[12]));
		Log.d("test321",String.valueOf(ha[13]));
		Log.d("test321",String.valueOf(ha[14]));
		Log.d("test321",String.valueOf(ha[15]));*/
		
		
		
		//ARObject.glCameraMatrix;
		/*
		Log.d("test", "yoooooooooo");
		Log.v("nonfree_jni_demo", "start runDemo");
		// Call the JNI interface
		Log.d("test123", "hello");
		Class<?> cla = someObject.getClass();
		
		try {
			Log.d("test123", "a1");
			Field filed = cla.getDeclaredField("glCameraMatrix");
			filed.setAccessible(true);
			//Log.d("test123", "yoooooooooooo");
			//glCameraMatrix = (float[]) filed.get(someObject);
			//Log.d("test123", String.valueOf(glCameraMatrix[0]));
			
			
			
		} catch (NoSuchFieldException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			Log.d("test123", "a2222");
			
		} catch (IllegalArgumentException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		*/
		
		//NonfreeJNILib.runDemo(glCameraMatrix);
		
		//System.out.println(glCameraMatrix.toString());
		
		/*
		float[] a = new float[16];
		a[0] = (float) 3.7;
		//float b = (float) 9.9;
		NonfreeJNILib.runDemo();
		//NonfreeJNILib.runDemo();
		
		startPreview();
		
		*/
	}
	
	  
	
	/**
	 * Inform the user about exceptions that occurred in background threads.
	 * This exception is rather severe and can not be recovered from.
	 * TODO Inform the user and shut down the application.
	 */
	@Override
	public void uncaughtException(Thread thread, Throwable ex) {
		Log.e("AndAR EXCEPTION", ex.getMessage());
		finish();
	}
	
	private void getData(){
		FileOutputStream fos = null;
		Bitmap b = this.takeScreenshot();
		Log.d("hey", Environment.getExternalStorageDirectory().toString ());
		
		try {
			fos = new FileOutputStream("/storage/extSdCard/nonfree/" + "GG02" + ".jpg");
			Log.d("hey", "find!");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			Log.d("hey", "GG!");
			e.printStackTrace();
		}
		if (fos != null) {
			b.compress(Bitmap.CompressFormat.JPEG, 90, fos);
			try {
				
				fos.flush();
				fos.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		/////////////
		/*
		try {
			Field dMWField = artoolkit.getClass().getDeclaredField("detectMarkerWorker");
			dMWField.setAccessible(true);
			try {
				DetectMarkerWorker tClass = (DetectMarkerWorker) dMWField.get(artoolkit);
				Field cFField= tClass.getClass().getDeclaredField("curFrame");
				cFField.setAccessible(true);
				byte[] tFrame = (byte[]) cFField.get(tClass);
				
				FileOutputStream outStream = null;
				try {
					outStream = new FileOutputStream(String.format(
							"/storage/extSdCard/nonfree/GOOD.jpg"));
					outStream.write(tFrame);
					outStream.close();
					Log.d("YO", "GOOOOOOOOOOOOOD");
				} catch (FileNotFoundException e) {
					e.printStackTrace();
				} catch (IOException e) {
					e.printStackTrace();
				} finally {
				}
				
			} catch (IllegalArgumentException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IllegalAccessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		
		} catch (NoSuchFieldException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		*/
		
		
		/*
		try {
			Field cameraField = AndARActivity.class.getDeclaredField("cameraHandler");
			cameraField.setAccessible(true);
			
			try {
				Log.d("YO","good60");
				camH = (CameraPreviewHandler) cameraField.get(this);
				Field convFid = camH.getClass().getDeclaredField("frame");
				convFid.setAccessible(true);
				
				byte[] tFrame;
				FileOutputStream outStream = null;
				try {
					outStream = new FileOutputStream(String.format(
							"/storage/extSdCard/nonfree/GOOD.jpg"));
					outStream.write(tFrame);
					outStream.close();
					Log.d("KK", "GOOOOOOOOOOOOOD");
				} catch (FileNotFoundException e) {
					e.printStackTrace();
				} catch (IOException e) {
					e.printStackTrace();
				} finally {
				}
				Log.d("YO","good AA");
			} catch (IllegalArgumentException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IllegalAccessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		} catch (NoSuchFieldException e) {
			// TODO Auto-generated catch block
			
			e.printStackTrace();
		}
		*/
		
		
	}
	
	private class LoadingDataAsyncTask extends AsyncTask<String, Integer, Integer>{

		@Override
		protected Integer doInBackground(String... param) {
			try {
				Thread.sleep(7000);
				Log.d("hey", "ready?");
				getData();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			return null;
		}

		@Override
		protected void onPostExecute(Integer result) {
			super.onPostExecute(result);
			//showData();
		}

		@Override
		protected void onProgressUpdate(Integer... values) {
			super.onProgressUpdate(values);
		}

		@Override
		protected void onPreExecute() {
			super.onPreExecute();
		}

	}
	
	
	
	
	
	
	private class CameraPreviewer implements PreviewCallback  {
		
		@Override
		public void onPreviewFrame(byte[] data, Camera camera) {
			// TODO Auto-generated method stub
			
			
			Log.d("YO", "GOOOOOOOOOOOOOD");
			/*FileOutputStream outStream = null;
			try {
				outStream = new FileOutputStream(String.format(
						"/storage/extSdCard/nonfree/GOOD.jpg"));
				outStream.write(data);
				outStream.close();
				Log.d("KK", "GOOOOOOOOOOOOOD");
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			} finally {
			}*/
			
			
		}

	}
	
}
