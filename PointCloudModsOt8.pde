
// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/


import org.openkinect.freenect.*;
import org.openkinect.processing.*;
import oscP5.*;        //  Load OSC P5 library

// Kinect Library object
Kinect kinect;
OscP5 oscP5; 

// OSC vars
int scalingFactor;
int maxScale;
int scalingRate;
int randomize = 0;
int xSkip = 4;
int ySkip = 4;
int dotWidth = 5;
int randomColor = 0;
int randomRed = round(random(0,255));
int randomGreen = round(random(0,255));
int randomBlue = round(random(0,255));
float xRotate = 0;
float yRotate = 0;
float xAngle = 0;
float yAngle = 0;

int pointMode = 0;

// randomizer (autopilot)
int randomCounter = 0;
int randomThreshold = 100;
int randomEverything;

// Angle for rotation
float a = 0;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

void setup() {
  // Rendering in P3D
  size(1440, 900, P3D);
  oscP5 = new OscP5(this, 8000);  // Start oscP5, listening for incoming messages at port 8000
  kinect = new Kinect(this);
  kinect.initDepth();

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
}

void draw() {
  
  if(randomEverything ==  1) {
    if(randomCounter > randomThreshold) {
      randomCounter = 0;
      scalingFactor = round(random(0,2000));
      maxScale = round(random(0,2000));
      scalingRate = round(random(0,500));
      //randomize = round(random(0,100));
      xSkip = round(random(4,20));
      ySkip = round(random(4,20));
      xRotate = round(random(2,20));
      yRotate = round(random(2,20));
      dotWidth = round(random(4,10));
    }
    randomCounter++;
    println("random counter " + randomCounter);
  }

  background(0);

  createPointCloud(width/2, height/2, 0);

  // Rotate
  //a += 0.03f;
  println("x rotate " + xRotate);
  if(xRotate > 10000000000000f) {
    xRotate = 0.0025f;
  }
  if(yRotate > 10000000000000f) {
    yRotate = 0.0025f;
  }
  xAngle += xRotate;
  yAngle += yRotate;
  
  // Scaling mod
  if(scalingFactor > maxScale) {
    scalingFactor = 200;
  }
  else {
    scalingFactor = scalingFactor + scalingRate;
  }
  
}

void createPointCloud(float xLoc, float yLoc, float zLoc) {
  
  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();
  
  //int skip = 6;
  
  // Translate and rotate
  translate(xLoc, yLoc, zLoc);
  //ROTATE!!
  rotateY(yAngle);
  rotateX(xAngle);
  
  if(randomColor == 1) {
    randomRed = round(random(0,255));
    randomGreen = round(random(0,255));
    randomBlue = round(random(0,255));
  }
    
  fill(randomRed,randomGreen,randomBlue);
  stroke(randomRed,randomGreen,randomBlue);

  for (int x = 0; x < kinect.width; x += xSkip) {
    for (int y = 0; y < kinect.height; y += ySkip) {
      int offset = x + y*kinect.width;

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);
      
      pushMatrix();
      // Scale up by 200
      float factor = scalingFactor;
      
      // Draw a point
      
      int ellipseX = 0 + round(random(0, randomize));
      int ellipseY = 0 + round(random(0, randomize));
      translate(v.x*factor + ellipseX, v.y*factor + ellipseY, factor-v.z*factor);
      //ellipse(ellipseX, ellipseY, dotWidth, dotWidth);
      //sphere(dotWidth);
      if(pointMode == 1) {
        point(ellipseX, ellipseY);
      }
      else {
        ellipse(ellipseX, ellipseY, dotWidth, dotWidth);
      }
      popMatrix();
    }
  }
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

void oscEvent(OscMessage theOscMessage) {   //  This runs whenever there is a new OSC message

  String addr = theOscMessage.addrPattern();  //  Creates a string out of the OSC message

  if (addr.indexOf("/fader9/scalingFactor") !=-1) {  
    scalingFactor = int(theOscMessage.get(0).floatValue()) * 20;
    println("sending scaling factor value " + scalingFactor);
  }
  if (addr.indexOf("/fader9/maxScale") !=-1) {  
    maxScale = int(theOscMessage.get(0).floatValue()) * 20;
    println("sending maxScale value " + maxScale);
  }
  if (addr.indexOf("/fader9/scalingRate") !=-1) {  
    scalingRate = int(theOscMessage.get(0).floatValue()) * 5;
    println("sending scalingRate value " + scalingRate);
  }
  if (addr.indexOf("/fader9/dotWidth") !=-1) {  
    dotWidth = int(theOscMessage.get(0).floatValue());
    println("dotWidth value " + dotWidth);
  }
  if (addr.indexOf("/fader9/randomize") !=-1) {  
    randomize = int(theOscMessage.get(0).floatValue());
    println("randomize value " + randomize);
  }
  if (addr.indexOf("/fader9/randomColor") !=-1) {  
    randomColor = int(theOscMessage.get(0).floatValue());
    println("random color " + randomColor);
  }
  if (addr.indexOf("/fader9/xSkip") !=-1) {  
    xSkip = int(theOscMessage.get(0).floatValue());
    println("x skip " + xSkip);
  }
  if (addr.indexOf("/fader9/ySkip") !=-1) {  
    ySkip = int(theOscMessage.get(0).floatValue());
    println("y skip " + ySkip);
  }
  if (addr.indexOf("/fader9/xRotate") !=-1) {  
    xRotate = theOscMessage.get(0).floatValue() * 10.0f;
    println("x rotate " + xRotate);
  }
  if (addr.indexOf("/fader9/yRotate") !=-1) {  
    yRotate = theOscMessage.get(0).floatValue() * 10.0f;
    println("y rotate " + yRotate);
  }
  if (addr.indexOf("/fader9/everythingRandom") !=-1) {  
    randomEverything = int(theOscMessage.get(0).floatValue());
  }
  if (addr.indexOf("/fader9/pointMode") !=-1) {  
    pointMode = int(theOscMessage.get(0).floatValue());
    println("pointMode " + pointMode);
  }
}
