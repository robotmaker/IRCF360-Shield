/** 
 *  Interface to robotmaker's 3D-360degree sensor (www.robotmaker.eu) 
 * by Colin Bacon. 7th June, 2011 
 * 
 * This program interfaces to the IRCF360 via an Arduino. It could also have been done directly with 
 * a Serial port level such as MAX232 
 * This routine just sends a command to the IRCF360 to start 360 degee proximity sensing routine
 * By clicking on the screen it will stop and start the command. 
 * The Sketch is a 3D environment and a floating disk represents direction of highest Proxmity reading.  
 * This is only approximate and no clever maths has been done. Each direction is checked and then averaged 
 * The closer you get to the IRCF360 the Z-Axis is effected and also the size
 * The disk will fly to the next reading received. 
 
 * This exmaple has is a combination of many of the example from the processing software
 * Credits go to all those who created the first examples

 * 
 * Note: This sketch assumes an Arduino and a ROBOTmaker 3D-360 sensor is connetect to the usb port
 

 */
 

import processing.serial.*;
 
 
int SB,Z1, Z2, Z3, Z4, Z5,EB;
int       idx, inByte, lost, packetErrors;
float x  ;
float y  ;

Serial myPort;                         // The serial port
int[] serialInArray = new int[5]; //Where we'll put what we receive


float  x1,x2, x3 , x4, x5;
float  dy1, dy2, dy3, dy4, dy5;
float  y1, y2, y3, y4, y5;
float targetY1, targetY2, targetY3, targetY4, targetY5, targetX;
float easing = 0.01;

String    myString = null;
int []    buffer = new int [8];
int []    channels = new int [18];
int       errors = 0;
boolean   failsafe = false;
long      last_refresh = 0;
float     ymag = 0;
float     newYmag = 0;
float     xmag = 0;
float     newXmag = 0; 
int borderSize = 10;

void setup() {
  size(340, 260, P2D); 
  frameRate(60);
  smooth(8);
  textAlign(CENTER);
  textSize(16);
  strokeWeight(2);  
 
 // Variable for drawing the cube
   
    
 // List all the available serial ports
    printArray(Serial.list());
 
 // Open the port you are using at the rate you want:
    myPort = new Serial(this, Serial.list()[1]); //Defaults at 9600baud
    myString = myPort.readStringUntil(6); //clean the first buffer
    println(myString);
    println(myPort); //Debug to show the port opened
}

void draw() {
    
  background(255);
  translate(30,-20); //Postion the dashboard in the middle of the canvas
  labelStyle();
  text("www.ROBOTmaker.eu", 101, 30); //Add Title to the Canvas
  text("PROXIMTY\nZONES", 101, 65); //Add Title to the Canvas

  //Draw the Canvas
        
        //Zone 1
        targetY1 = Z1*2  ;
        dy1 = targetY1 - y1;
        y1 += dy1 * easing;
        barLabelStyle(); //Line Text
        text( "Z1",10,105);
        barBgStyle();//Line Style
        line(20, 100, y1, 100); //<>//
        barStyle(); //Line Ends
        line(y1, 95, y1, 105);
        barLabelStyle(); //Line Text
        text(nf(Z1, -11, 0), 230, 105);
        
        //Zone 2
        targetY2 = Z2*2 ;
        dy2 = targetY2 - y2;
        y2 += dy2 * easing;
        barLabelStyle(); //Line Text
        text( "Z2",10,125);
        barBgStyle();//Line Style
        line(20, 120, y2, 120);
        barStyle(); //Line Ends
        line(y2, 125, y2, 115);
        barLabelStyle(); //Line Text
        text(nf(Z2, -11, 0), 230, 125);
        
        //Zone 3 //<>//
        targetY3 =  Z3*2 ;
        dy3 = targetY3 - y3; //<>//
        y3 += dy3 * easing;
        barLabelStyle(); //Line Text
        text( "Z3",10,145);
        barBgStyle();//Line Style
        line(20, 140, y3, 140);
        barStyle(); //Line Ends
        line(y3, 135, y3, 145);
        barLabelStyle(); //Line Text
        text(nf(Z3, -11, 0), 230, 145);
        
         
        //Zone 4
        targetY4 = Z4*2 ; //<>//
        dy4 = targetY4 - y4;  //<>//
        y4 += dy4 * easing;  //<>//
        barLabelStyle(); //Line Text
        text( "Z4",10,165);
        barBgStyle();//Line Style
        line(20, 160, y4, 160);
        barStyle(); //Line Ends
        line(y4, 155, y4, 165);
        barLabelStyle(); //Line Text
        text(nf(Z4, -11, 0), 230, 165);
        
         //<>//
        //Zone 5
        targetY5 = Z5*2 ; //<>//
        dy5 = targetY5 - y5;
        y5 += dy5 * easing;
        barLabelStyle(); //Line Text
        text( "Z5",10,185);
        barBgStyle();//Line Style
        line(20, 180, y5, 180); 
        barStyle(); //Line Ends
        line(y5, 175, y5, 185);
        barLabelStyle(); //Line Text
        text(nf(Z5, -11, 0), 230, 185);
    
  //Check data from serial port arriving from Arduino//
  //The package from the Arduino is 7 bytes long as follows:
  // 025 = StartByte
  // Zone 1  (Values 0-100)
  // Zone 2
  // Zone 3
  // Zone 4
  // Zone 5
  // 0 = End Byte
  
  
  if (myPort.available() == 0){   
    
   //If no incoming data then count lost packages
    packetErrors++; 
    textSize(13); //set text size
    noStroke();
    
   //fill(120);
    //text("STANDARD\nPARAMETRIZATION", 120, 310);
    text ("HANDSHAKING", 95,237);
    text (packetErrors, 162,237);
   }
   
   
   
   //Start of Handshaking part 
   //Check for incoming Serial Port Data
   //When data is comming in then check for a start byte No.1=B11110000 and stop byte No.6=B00000000 
       while (myPort.available() > 0)  
   {
      inByte = myPort.read();                 
   
  //Read the Byte that just came in
  //if it's a new packet and the start byte is not  B00001111 (DEC15) then it's an error. 
    
  if (idx == 0 && inByte != 0x0F) // error - if no data in buffer and 1st inByte is not a start byte (56) then wait for the start byte 
      {          
        barLabelStyle();
        text("Package Error", 0,0);
        println("Searching start- & end-byte"); 
      } 
  // if it's a new packet and the start byte is  B00001111 (DEC15) then start reading the next 6 bytes. Zone 1-5 + stop byte  
  else 
      { 
        buffer[idx++] = inByte;  // fill the buffer with 25 Bytes 
      }
  // if the buffer of 6 Bytes is reached then start to decode    //<>// //<>// //<>//
  if (idx == 7) 
      {  
        idx = 0;  //reset the buffer count for the next cycle
        if (buffer[7] != 0x00) //Check that the packet size is 7 bytes long with stop byte b00000000 as the last byte
              {  
              errors++;                //Count the number of errors
              print("Package Length Error <> 7. Check Arduino Connection");
              println(errors);         //Print to the error totals to the console
              } 
      else
      println("found package"); 
  
  
  //End of Handshaking part //<>// //<>// //<>//
      { //Start decoding the bits and bytes
        //Buffer[0] contains the start byte value of 15 
        //Buffer[6] contains the end byte value of 0 
        //Each Direction is a separate channel
        
        
        

                  SB = buffer[0]; //Start Bytes
                  Z1 = buffer[1];
                  Z2 = buffer[2];
                  Z3 = buffer[3]; //<>// //<>// //<>//
                  Z4 = buffer[4];
                  Z5 = buffer[5];
                  EB = buffer[6]; //End Bytes
                  packetErrors = 0; //reset package waiting indicator
              channels[0]  = (buffer[1]); //The first Channel North      
                  // print the values (for debugging purposes only):
                  println(Z1 + "\t" + Z2 + "\t" + Z3 + "\t" + Z4 + "\t" + Z5 + "\t");
      
      
      
        
      }     
    }
  }
}

// Styles -----

void curveStyle() {
  stroke(170);
  noFill();
}

void labelStyle() {
  noStroke();
  fill(120);
}

void circleStyle() {
  noStroke();
  fill(0);
}

void barBgStyle() {
  stroke(220);
  noFill();
}

void barStyle() {
  stroke(50);
  noFill();
}

void barLabelStyle() {
  noStroke();
  fill(120);
}