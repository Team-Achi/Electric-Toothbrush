
// import org.json.JSONObject;
import processing.serial.*;

JSONObject json;
Serial myPort;

boolean ready = false;
int x = 0;
int y = 0;
int z = 0;

void setup()
{
  size(600, 600, P3D);
  
  println(Serial.list()); 
  
  myPort = new Serial(this, Serial.list()[2], 9600); 
  myPort.bufferUntil(10);
}

void draw()
{
  lights();
  
  background(0);
  
  if(ready) {
    fill(255);
    textSize(16);
    text("X: " + x, 10, 20);
    text("Y: " + y, 10, 40);
    text("Z: " + z, 10, 60);
    
    pushMatrix();
      fill(255, 0, 0);
      translate(width/2, height/2, 0);
      rotateX(radians(-x));
      rotateY(radians(z));
      rotateZ(radians(y));
      box(200);
    popMatrix();
  } else {
    fill(255);
    textSize(20);
    text("Calibrating, don't move...", 10, 80);
  }
}

void serialEvent(Serial port)
{
  String str = port.readString();  
  
  str = str.substring(0, str.length() - 1);
  println(str);
  
  if(str.charAt(0) == '{') 
  {
    ready = true;    
    
    str = str.substring(1, str.length() - 2);
    // println("str=" + str);
    
    String[] ar = splitTokens(str, ":,");
    
    x = Integer.parseInt(ar[1]);
    y = Integer.parseInt(ar[3]);
    z = Integer.parseInt(ar[5]);
  } 
  else 
  {
    ready = false;
    println(str);
  }  
}
