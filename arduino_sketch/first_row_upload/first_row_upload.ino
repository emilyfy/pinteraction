/*  pinteraction
 *  sketch for first row arduino mega, in serial communication with the other arduino mega controlling 10 motorized linear potentiometers
 *  
 *  receives info about feedback for equalizer mode (mode 2)
 *  through rosserial publish on topic /feedback on whether it is being turned off
 *  
 *  read the proximity sensor on the glove for visual mode (mode 3)
 *  through rosserial publish on topic /distance the distance from the glove to pins
 */

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <pinteraction/BoolMultiArray.h>

#define pins 10

unsigned int currTime, commTime = 0;

const int sensorPin = A0;

int mode = 0;

ros::NodeHandle nh;

boolean fdb[pins] = {true, true, true, true, true, true, true, true, true, true};
pinteraction::BoolMultiArray fdb_msg;
ros::Publisher pub_fdb("feedback", &fdb_msg);
std_msgs::UInt16 dst_msg;
ros::Publisher pub_dst("distance", &dst_msg);

void setup()
{
  pinMode(sensorPin, INPUT);

  fdb_msg.data_length = pins;
  fdb_msg.data = fdb;

  Serial3.begin(115200);
  nh.initNode();
  nh.advertise(pub_dst);
  nh.advertise(pub_fdb);
}

void loop() {
  currTime = millis();

  if (mode==2) checkFdb();

  if (currTime > commTime) {
    nh.getParam("mode", &mode);
    if (mode==3) sendGlove();
    commTime = commTime + 100;
  }

}

void checkFdb() {

  static int count = 0;
  static int i = 0;
  static char rcv[pins];
  while(Serial3.available())
  {
    unsigned char ch = Serial3.read();
    switch(count)
    {
      case 0:
        if(ch==0xFA)
          count++;
        else
          count=0;
        break;
      case 1:
        if(ch==0xBC)
        {
          i=0;
          count++;
        }
        else
          count=0;
        break;
      case 2:
        rcv[i]=ch;
        i++;
        if(i>=pins){
          i=0;
          count++;
        }
        break;
      case 3:
        if(ch==0xDE)
          count++;
        else
          count=0;
        break;
      case 4:
        for (int j=0;j<pins;j++) {
          if (rcv[j]) fdb[j]=true;
          else fdb[j]=false;
        }
        count=0;
        fdb_msg.data = fdb;
        pub_fdb.publish( &fdb_msg );
        nh.spinOnce();
        break;
       
      default:
        count=0;
        break;     
    }
  }
  
}

void sendGlove(void) {
  dst_msg.data = analogRead(sensorPin);
  pub_dst.publish( &dst_msg );
  nh.spinOnce();
}
