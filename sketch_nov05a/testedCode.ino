#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <TinyGPS++.h>
#include <Keypad.h>

#define MAX 15
#define TEMP 0
#define PERM 1
#define infinity 999
#define inf 9999

DFRobotDFPlayerMini myDFPlayer;

void printDetail(uint8_t type, int value);

float info[15][15] = {{1, 28.594086, 77.020211},
  {2, 28.594194, 77.019496},
  {3, 28.594746, 77.019657},
  {4, 28.594709, 77.019022},
  {5, 28.595239, 77.019087},
  {6, 28.595202, 77.01958},
  {7, 28.59496, 77.01824},
  {8, 28.595352, 77.017413},
  {9, 28.593993, 77.017118},
  {10, 28.594957, 77.017141},
  {11, 28.594785, 77.020944},
  {12, 28.594873, 77.021721}
};

const byte ROWS = 4; 
const byte COLS = 4;
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};


int numarray[6] = { -1, -1, -1, -1, -1, -1}; //checksum -(36 71 78 82 77 67) = -411
int i = 0;
String response = "";
String toprint = "";
int comma[10]; 
byte rowPins[ROWS] = {9, 8, 7, 6}; 
byte colPins[COLS] = {5, 4, 3, 2}; 
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
byte gps;
int  sum = 0;
double lat_deg, long_deg,lat_min, long_min;
int n = 12;
double minsum, usrbrng, destbrng ;
int path[MAX];
int lastnode = 0;
int nodeno = 0;
String current_time;
double current_lat ;
double current_long ;
String current_alt ;
double  next_lat, next_long;
double lasttime_lat, lasttime_long;
double brng1 ;
int nextnodeno;
int hrhand;
int loopcount = 0;
static unsigned long counter = 0;
int adj[MAX][MAX] =  {
  {0,   0,  0,    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, },
  {0,   0,  5,  inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, },
  {0,   5,  0,    5, inf, inf, inf, inf, inf, 200, inf, inf, inf, },
  {0, inf,  5,    0,   5,   5,   5, inf, inf, inf, inf,   5, inf, },
  {0, inf, inf,   5,   0,   5, inf,   5, inf, inf, inf, inf, inf, },
  {0, inf, inf,   5,   5,   0,   5,  11,   5, inf, inf, inf, inf, },
  {0, inf, inf,   5, inf,   5,   0, inf, inf, inf, inf, inf, inf, },
  {0, inf, inf, inf,   5,  11, inf,   0,   5, inf,   5, inf, inf, },
  {0, inf, inf, inf, inf,   5, inf,   5,   0, inf,   5, inf, inf, },
  {0, inf, 200, inf, inf, inf, inf, inf,   0, inf,  20, inf, inf, },
  {0, inf, inf, inf, inf, inf, inf,   5,   5,  20,   0, inf, inf, },
  {0, inf, inf,   5, inf, inf, inf, inf, inf, inf, inf,   0,   5, },
  {0, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf,   5,   0, }
};


struct node
{
  int predecessor;
  int dist; /*minimum distance of node from source*/
  int status;

};

int findclosenode(float lat1, float lon1)
{

  int k;
  minsum = 999;
  double ans;
  int imin;
  double latsq, longsq;


  for (k = 1; k <= n; k++)
  { ans = 0;
    latsq =  fabs(lat1 - (info[k - 1][1]));

    longsq = fabs(lon1 - (info[k - 1][2]));

    ans = longsq + latsq;

    if (ans <= minsum )
    {
      minsum = ans;
      imin = k;
    }
    int mindist = 100;
    minsum *= 1000000;
    //Serial.print("Ans value is : ");
    //Serial.println(minsum);
    if (minsum <= mindist)
    {
      Serial.print("You are close to Node no. :" );
      delay(200); myDFPlayer.play(41);
      delay(1900);
      Serial.println(imin);

      lastnode = imin;
    }
  }
  return lastnode;
}
int findnodeno(float lat1, float lon1)
{

  int k;
  minsum = 999;
  double ans;
  int imin;
  double latsq, longsq;

  for (k = 1; k <= n; k++)
  {
    latsq =  fabs(lat1 - (info[k - 1][1]));
    longsq = fabs(lon1 - (info[k - 1][2]));
    ans = longsq + latsq;

    if (ans <= minsum )
    {
      minsum = ans;
      imin = k;
    }
  }
  return imin;
}
int flag = 0;

void display()
{
  int i, j;
  for (i = 1; i <= n; i++)
  {
    for (j = 1; j <= n; j++)
    {
    }
    //Serial.println("");
  }

}

int findpath(int s, int d, int *sdist)
{
  struct node state[MAX];
  int i, min, count = 0, current, newdist, u, v;
  *sdist = 0;
  /* Make all nodes temporary */
  for (i = 1; i <= n; i++)
  {
    state[i].predecessor = 0;
    state[i].dist = infinity;
    state[i].status = TEMP;
  }

  /*Source node should be permanent*/
  state[s].predecessor = 0;
  state[s].dist = 0;
  state[s].status = PERM;

  /*Starting from source node until destination is found*/
  current = s;
  while (current != d)
  {
    for (i = 1; i <= n; i++)
    {
      /*Checks for adjacent temporary nodes */
      if ( adj[current][i] > 0 && state[i].status == TEMP )
      {
        newdist = state[current].dist + adj[current][i];
        /*Checks for Relabeling*/
        if ( newdist < state[i].dist )
        {
          state[i].predecessor = current;
          state[i].dist = newdist;
        }
      }
    }/*End of for*/

    /*Search for temporary node with minimum distand make it current
      node*/
    min = infinity;
    current = 0;
    for (i = 1; i <= n; i++)
    {
      if (state[i].status == TEMP && state[i].dist < min)
      {
        min = state[i].dist;
        current = i;
      }
    }/*End of for*/

    if (current == 0) /*If Source or Sink node is isolated*/
      return 0;
    state[current].status = PERM;
  }/*End of while*/

  /* Getting full path in array from destination to source */
  while ( current != 0 )
  {
    count++;
    path[count] = current;
    Serial.print(" <-- ");
    //Serial.println(String(count));
    //Serial.print(String(path[count]));
    switch (current) {
      case 1  : Serial.print("A block"); break;
        delay(400);
      case 2 : Serial.print(" B block"); break;
        delay(400);
      case 7 : Serial.print(" Canteen"); break;
        delay(400);
      case 3 : Serial.print(" Library"); break;
        delay(400);
      case 4 : Serial.print(" C block"); break;
        delay(400);
      case 5 : Serial.print(" D block"); break;
        delay(400);
      case 6 : Serial.print(" E block"); break;
        delay(400);
      case 8 : Serial.print(" Girls Hostel"); break;
        delay(400);
      case 12 : Serial.print(" Campus Entry Gate"); break;
        delay(400);
      case 9 : Serial.print("Indian Bank"); break;
        delay(400);
      case 10 : Serial.print(" Boys Hostel"); break;
        delay(400);
      case 11 : Serial.print("Admin Block"); break;
        delay(400);


    }

    current = state[current].predecessor;
    if ( current != 0 )
      Serial.print(" ");
  }

  /*Getting distance from source to destination*/
  for (i = count; i > 1; i--)
  {
    u = path[i];
    v = path[i - 1];
    *sdist += adj[u][v];
  }
  return (count);
}/*End of findpath()*/



void setup()
{
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial2.begin(9600);
  if (!myDFPlayer.begin(Serial3)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true);
  }
  myDFPlayer.volume(18);
  portal_beginning();
  while (1) {
    sendData();  //Sets GPS Mode
     if (current_lat >= 28.00 && current_long >= 76.00) { //valid string
    break;
  }
  }
  getsource(current_lat, current_long);

}

void loop()
{

  /* static unsigned long timer = millis();

    if (millis() - timer > 3000) {
     timer = millis();
     myDFPlayer.next();  //Play next mp3 every 3 second.
    }
  */
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
  while(1){
  sendData();
 if(current_lat!=lasttime_lat||current_long!=lasttime_long)
 break;
  }
  delay(500);
  Serial.println("");
  findclosenode(current_lat, current_long);
  nextnodeno = findnextnode(lastnode);
  Serial.print(" Next Node is: ");
  Serial.println(nextnodeno);
  delay(200);
   switch (nextnodeno) {
    case 1  : Serial.println("A block"); myDFPlayer.play(32);
      delay(1600);
      break;
    case 2 : Serial.println(" B block"); myDFPlayer.play(25);
      delay(1600);
      break;
    case 7 : Serial.println(" Canteen"); myDFPlayer.play(36);
      break;
      delay(1600);
    case 3 : Serial.println(" Library"); myDFPlayer.play(23);
      delay(1600);
      break;

    case 4 : Serial.println(" C block"); myDFPlayer.play(24);
      delay(1600);
      break;

    case 5 : Serial.println(" D block"); myDFPlayer.play(34);
      delay(1600);
      break;

    case 6 : Serial.println(" E block"); myDFPlayer.play(30);
      delay(1600);
      break;

    case 8 : Serial.println(" Girls Hostel"); myDFPlayer.play(38);
      delay(1600);
      break;
    case 12 : Serial.println(" Campus Entry Gate"); myDFPlayer.play(35);
      delay(1600);
      break;

    case 9 : Serial.println("Indian Bank"); myDFPlayer.play(26);
      delay(1600);
      break;
    case 10 : Serial.println(" Boys Hostel"); myDFPlayer.play(25);
      delay(1600);
      break;

    case 11 : Serial.println("Admin Block"); myDFPlayer.play(31);
      delay(1600);
      break;

   }
  Serial.print("\n Last Node Node is: ");
  Serial.println(lastnode);
  switch (lastnode) {
    case 1  : Serial.println("A block"); myDFPlayer.play(32);
      delay(1600);
      break;
    case 2 : Serial.println(" B block"); myDFPlayer.play(25);
      delay(1600);
      break;
    case 7 : Serial.println(" Canteen"); myDFPlayer.play(36);
      break;
      delay(1600);
    case 3 : Serial.println(" Library"); myDFPlayer.play(23);
      delay(1600);
      break;

    case 4 : Serial.println(" C block"); myDFPlayer.play(24);
      delay(1600);
      break;

    case 5 : Serial.println(" D block"); myDFPlayer.play(34);
      delay(1600);
      break;

    case 6 : Serial.println(" E block"); myDFPlayer.play(30);
      delay(1600);
      break;

    case 8 : Serial.println(" Girls Hostel"); myDFPlayer.play(38);
      delay(1600);
      break;
    case 12 : Serial.println(" Campus Entry Gate"); myDFPlayer.play(35);
      delay(1600);
      break;

    case 9 : Serial.println("Indian Bank"); myDFPlayer.play(26);
      delay(1600);
      break;
    case 10 : Serial.println(" Boys Hostel"); myDFPlayer.play(25);
      delay(1600);
      break;

    case 11 : Serial.println("Admin Block"); myDFPlayer.play(31);
      delay(1600);
      break;

   }
  delay(200);
  if (nextnodeno != -1) {
    next_lat = info[nextnodeno - 1][1];
    next_long = info[nextnodeno - 1][2];

    Serial.print("\n Angle between Current lat, long (");
     Serial.print(current_lat*100000);
     Serial.print(",");
     Serial.print(current_long*100000);
     Serial.print(") and Next Node lat,long (");
     Serial.print(next_lat*100000); Serial.print(","); Serial.print(next_long*100000); Serial.print(") is ");
    destbrng = angleFromCoordinate(current_lat, current_long,
                                   next_lat, next_long); //next_long

    Serial.print(destbrng);
    double distance = finddistance(current_lat, current_long,
                                   next_lat, next_long); //next_lat
    Serial.print("\n Distance from Next Node is ");
    Serial.println(distance);
  }
  else
  {
    Serial.println("You have arrived at your destination");
  }
 
  if (loopcount % 1 == 0) {
     Serial.print("\n Angle between lasttime lat, long (");
     Serial.print(lasttime_lat*100000);
     Serial.print(",");
     Serial.print(lasttime_long*100000);
     Serial.print(") and current Node lat,long (");
     Serial.print(current_lat*100000); Serial.print(","); Serial.print(current_long*100000); Serial.print(") is ");
    usrbrng = angleFromCoordinate(lasttime_lat, lasttime_long, current_lat, current_long);
      if(destbrng-usrbrng<0)
          brng1 = destbrng - usrbrng + 360;  
      else
          brng1 = destbrng - usrbrng;                           
     Serial.print("Difference between the 2 angles is ");
    Serial.println(brng1);
    lasttime_lat =  current_lat;
    lasttime_long = current_long;
    brng1 = brng1 / 30;
    
    hrhand = brng1;
    brng1 = brng1 - hrhand;
  if (brng1 >= 0.5)
    hrhand++;
  Serial.print("\n Turn to ");
  Serial.print(hrhand);
  Serial.print(" O' Clock");

    loopcount = 0;
  }
  loopcount++;
  delay(100);
}
void playnum(int num) {
  switch (num) {
    case 12  : delay(400); myDFPlayer.play(17);
      delay(1900); break;


    case 11 :  delay(400); myDFPlayer.play(2);
      delay(1900); break;

    case 8 :  delay(400);
      myDFPlayer.play(20);
      delay(1900); break;

    case 3 : delay(400); myDFPlayer.play(7);
      delay(1900); break;

    case 1 : delay(400); myDFPlayer.play(13);
      delay(1900); break;

    case 2 : delay(400); myDFPlayer.play(4);
      delay(1900); break;

    case 4 : delay(400); myDFPlayer.play(22);
      delay(1900); break;

    case 5 :  delay(400); myDFPlayer.play(6);
      delay(1900); break;

    case 9 :  delay(400); myDFPlayer.play(14);
      delay(1900); break;

    case 7 : delay(400); myDFPlayer.play(10);
      delay(1900); break;

    case 10 :    delay(400); myDFPlayer.play(1);
      delay(1900); break;

    case 6 :  delay(400); myDFPlayer.play(8);
      delay(1900); break;

  }

}


double finddistance(float lat1, float lon1, float lat2, float lon2) {

  double earthRadiusKm = 6371;
  double dLat = (lat2 - lat1) * 0.0174533;
  double dLon = (lon2 - lon1) * 0.0174533;

  lat1 = lat1 * 0.0174533;
  lat2 = lat2 * 0.0174533;

  double a = pow(sin(dLat / 2.0), 2) + cos(lat1) * cos(lat2) *
             pow(sin(dLon / 2.0), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return earthRadiusKm * c;
}

int findnextnode(int last) {
  int i;

  // Serial.println(nodeno);
  for (i = nodeno ; i >= 2 ; i--)
  {
    if (path[i] == last) {
      return path[i - 1];
    }
  }
  if (path[i] == last)
    return -1;
}
double angleFromCoordinate(float lat1, float long1, float lat2, float long2) {

  long int lati1, longi1, lati2, longi2;
  lati1 = lat1 * 1000000;

  longi1 = long1 * 1000000;

  lati2 =  lat2 * 1000000;

  longi2 = long2 * 100000;
 
  double dLon = (long2 - long1);

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

  double brng = atan2(y, x);

  brng = brng * 57296 / 1000;

  //String side = "Right";

  brng = (brng + 360);//
  brng = fmod(brng, 360);
  brng = 360 - brng; // count degrees counter-clockwise - remove to make clockwise
  Serial.println(brng);
 
  if (millis() - counter > 3000) {
    counter = millis();
    myDFPlayer.play(40);
    delay(1600);
    playnum(hrhand);
    myDFPlayer.play(21);
    delay(1600);
  }
  delay(1000);
  return brng;
}
void getsource(float latitude, float longitude)
{
  int i, j;
  int source, dest;

  int shortdist, count;
  float lat, lon;
  source = findnodeno(latitude, longitude);

  lastnode = source;
  lasttime_lat = current_lat;
  lasttime_long = current_long;
  Serial.print("Your Current location is ");
  myDFPlayer.play(33);
  delay(1900);

  switch (source) {
    case 12  : Serial.println(" Campus Entry Gate");
      delay(400); myDFPlayer.play(35);
      (1900); break;


    case 11 : Serial.println(" Admin Block"); delay(400); myDFPlayer.play(31);
      delay(1900); break;

    case 8 : Serial.println(" Girls Hostel"); delay(400);
      myDFPlayer.play(38);
      delay(1900); break;

    case 3 : Serial.println(" Library"); delay(400); myDFPlayer.play(23);
      delay(1900); break;

    case 1 : Serial.println(" A block"); delay(400); myDFPlayer.play(32);
      delay(1900); break;

    case 2 : Serial.println(" B block"); delay(400); myDFPlayer.play(37);
      delay(1900); break;

    case 4 : Serial.println(" C block"); delay(400); myDFPlayer.play(24);
      delay(1900); break;

    case 5 : Serial.println(" D block"); delay(400); myDFPlayer.play(34);
      delay(1900); break;

    case 9 : Serial.println(" Indian Bank"); delay(400); myDFPlayer.play(26);
      delay(1900); break;

    case 7 : Serial.println(" Canteen"); delay(400); myDFPlayer.play(36);
      delay(1900); break;

    case 10 : Serial.println(" Boys Hostel");   delay(400); myDFPlayer.play(25);
      delay(1900); break;

    case 6 : Serial.println(" E block");  delay(400); myDFPlayer.play(30);
      delay(1900); break;

  }


  dest = portal_menu();
  if (source == 0 || dest == 0)
  { Serial.println("Invalid Node");
  }

  count = findpath(source, dest, &shortdist);
  nodeno = count;
  /*End of while*/
}

void sendData(){
  if (Serial2.available() > 0) {
  gps = Serial2.read();
  shift(int(gps));
  sum = numarray[0] + numarray[1] + numarray[2] + numarray[3] + numarray[4] + numarray[5] - 411;
  if (sum == 0) {
    response = "";
   // Serial.print("toprint");
    while (1) {
      if (Serial2.available() > 0) {
    
   
        byte data = Serial2.read();
        char d = int(data);
       // Serial.print(d);
        response += d;
        if (d == '$')
          { 
            break;
          }
          
          }
    }
    Serial.println(response);
   // Serial.println(response.length());
    if(response.length()>=55)
      {
        comma[0] = response.indexOf(',');
        for(int i=1;i<6;i++){
             comma[i] = response.indexOf(',', comma[i-1] + 1 );
         }
      delay(100);
      String lati="";
      String longi="";
      lati = response.substring(comma[2]+1 ,comma[3]);
      longi = response.substring(comma[4]+1,comma[5]);

      //Serial.println(response);
       lat_deg = lati.substring(0,2).toFloat();
       lat_min = lati.substring(2).toFloat(); 
       
       long_deg = longi.substring(0,3).toFloat();
       long_min = longi.substring(3).toFloat();

    lat_deg += lat_min/60;    
    long_deg += long_min/60;
    current_lat=lat_deg;
    current_long=long_deg;
    /*Serial.print("Latitiude");
    Serial.println(lat_deg* 10000);
    Serial.print("  Longitude");
    Serial.println(long_deg*10000); */
      }
  }
}
}
//Serial.print(numarray);


void shift(int byt) {
  for (int j = 0; j < 5 ; j++)
  {
    numarray[j] = numarray[j + 1];
  }
  char a = byt;
  toprint = toprint.substring(1) + a ;

  numarray[5] = byt;
}
void portal_beginning(void)
{

  Serial.flush();

  Serial.println("                             G.G.S.I.P.U. BLIND PEDESTRIAN NAVIGATION SYSTEM");

  myDFPlayer.play(39);
  delay(4000);
  Serial.println("Please WAIT while we Fetch your Current Location");
  myDFPlayer.play(43);
  delay(4000);
}

int portal_menu(void)
{

  Serial.flush();
  delay(1300);
  Serial.println("Choose Your Destination:-");
  myDFPlayer.play(28);
  delay(1900);


  Serial.println(" 1. A block");
  myDFPlayer.play(13);
  delay(900);
  myDFPlayer.play(32);
  delay(1600);
  Serial.println(" 2. B block");
  myDFPlayer.play(4);
  delay(900);
  myDFPlayer.play(37);
  delay(1600);
  Serial.println(" 3. Library");
  myDFPlayer.play(7);
  delay(900);
  myDFPlayer.play(23);
  delay(1600);

  Serial.println(" 4. C block");
  myDFPlayer.play(22);
  delay(900);
  myDFPlayer.play(24);
  delay(1600);
  Serial.println(" 5. D block");
  myDFPlayer.play(6);
  delay(900);
  myDFPlayer.play(34);
  delay(1600);
  Serial.println(" 6. E block");
  myDFPlayer.play(8);
  delay(900);
  myDFPlayer.play(30);
  delay(1600);
  Serial.println(" 7. Canteen");
  myDFPlayer.play(10);
  delay(900);
  myDFPlayer.play(36);
  delay(1600);
  Serial.println(" 8. Girls Hostel");
  myDFPlayer.play(20);
  delay(900);
  myDFPlayer.play(38);
  delay(1600);
  Serial.println(" 9. Indian Bank");
  myDFPlayer.play(14);
  delay(900);
  myDFPlayer.play(26);
  delay(1600);
  Serial.println(" 10.  Boys Hostel");
  myDFPlayer.play(1);
  delay(900);
  myDFPlayer.play(25);
  delay(1600);
  Serial.println(" 11. Admin Block");
  myDFPlayer.play(2);
  delay(900);
  myDFPlayer.play(31);
  delay(1600);

  Serial.println(" 12. Campus Entry Gate");
  myDFPlayer.play(17);
  delay(900);
  myDFPlayer.play(35);
  delay(1600);
  Serial.print("Your Selected Destination is : ");
  myDFPlayer.play(3);
  delay(1600);
  char* ipStr="";
  char customKey = customKeypad.getKey();
  while (customKey!='#') {
    ipStr=ipStr+customKey;
    customKey = customKeypad.getKey();
    }

  int option1 = atoi(ipStr);
  Serial.println(option1);
  switch (option1) {
    case 1  : Serial.println("A block"); myDFPlayer.play(32);
      delay(1600);
      break;
    case 2 : Serial.println(" B block"); myDFPlayer.play(25);
      delay(1600);
      break;
    case 7 : Serial.println(" Canteen"); myDFPlayer.play(36);
      break;
      delay(400);
    case 3 : Serial.println(" Library"); myDFPlayer.play(23);
      delay(1600);
      break;

    case 4 : Serial.println(" C block"); myDFPlayer.play(24);
      delay(1600);
      break;

    case 5 : Serial.println(" D block"); myDFPlayer.play(34);
      delay(1600);
      break;

    case 6 : Serial.println(" E block"); myDFPlayer.play(30);
      delay(1600);
      break;

    case 8 : Serial.println(" Girls Hostel"); myDFPlayer.play(38);
      delay(1600);
      break;
    case 12 : Serial.println(" Campus Entry Gate"); myDFPlayer.play(35);
      delay(1600);
      break;

    case 9 : Serial.println("Indian Bank"); myDFPlayer.play(26);
      delay(1600);
      break;
    case 10 : Serial.println(" Boys Hostel"); myDFPlayer.play(25);
      delay(1600);
      break;

    case 11 : Serial.println("Admin Block"); myDFPlayer.play(31);
      delay(1600);
      break;


  }
  return (option1);
}

void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F(""));
      Serial.print(value);
      Serial.println("");
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
//********************************************************************************************************************************************************************

