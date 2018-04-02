#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#define MAX 15
#define TEMP 0
#define PERM 1
#define infinity 999
#define inf 9999

//SoftwareSerial Serial2(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

float info[15][15] = {{1,28.594086,77.020211},
                        {2,28.594194,77.019496},
                        {3,28.594746,77.019657},
                        {4,28.594709,77.019022},
                        {5,28.595239,77.019087},
                        {6,28.595202,77.01958},
                        {7,28.59496,77.01824},
                        {8,28.595352,77.017413},
                        {9,28.593993,77.017118},
                        {10,28.594957,77.017141},
                        {11,28.594785,77.020944},
                        {12,28.594873,77.021721}};


int n = 12;
double minsum,usrbrng,destbrng ;
int path[MAX];
int lastnode = 0;
int nodeno = 0;
String current_time;
String current_lat ;
String current_long ;
String current_alt ;
float  next_lat,next_long;
String lasttime_lat, lasttime_long;
int nextnodeno;
int loopcount = 0;
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
   double latsq,longsq;
   

   for(k=1;k<=n;k++)
     {  ans = 0;
         latsq =  fabs(lat1 - (info[k-1][1]));

         longsq = fabs(lon1 - (info[k-1][2]));
        
         ans = longsq + latsq;
       
     if (ans <= minsum )
       {
          minsum = ans;
          imin=k;
       }
        int mindist = 200;
          minsum*= 1000000;
          //Serial.print("Ans value is : ");
          //Serial.println(minsum);
            if(minsum <= mindist)
            {
              Serial.print("You are close to Node no. :" );
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
   double latsq,longsq;

   for(k=1;k<=n;k++)
     {
         latsq =  fabs(lat1 - (info[k-1][1]));
         /*Serial.print("---------");
          latsq *= 10000; 
         Serial.println(latsq);
         delay(200);*/
         //printf("%f\t",latsq);

         longsq = fabs(lon1 - (info[k-1][2]));
         /* Serial.print("---------");
          longsq *= 10000; 
         Serial.println(longsq);*/
        // printf("%f\t",longsq);

         ans = longsq + latsq;
        /*  Serial.println("----");
        Serial.println(ans);
         Serial.println("----");
         Serial.println(""); 
   Serial.println("");*/

     if (ans <= minsum )
       {
          minsum = ans;
          imin=k;
       }
   }
   return imin;
}
int flag = 0;

void display()
{
int i,j;
for(i=1;i<=n;i++)
{
for(j=1;j<=n;j++)
{
//Serial.print("   ");
//Serial.print(adj[i][j]);
//Serial.print("   ");
}
//Serial.println("");
}

}

int findpath(int s,int d,int *sdist)
{
struct node state[MAX];
int i,min,count=0,current,newdist,u,v;
*sdist=0;
/* Make all nodes temporary */
for(i=1;i<=n;i++)
{
state[i].predecessor=0;
state[i].dist = infinity;
state[i].status = TEMP;
}

/*Source node should be permanent*/
state[s].predecessor=0;
state[s].dist = 0;
state[s].status = PERM;

/*Starting from source node until destination is found*/
current=s;
while(current!=d)
{
for(i=1;i<=n;i++)
{
/*Checks for adjacent temporary nodes */
if ( adj[current][i] > 0 && state[i].status == TEMP )
{
newdist=state[current].dist + adj[current][i];
/*Checks for Relabeling*/
if( newdist < state[i].dist )
{
state[i].predecessor = current;
state[i].dist = newdist;
}
}
}/*End of for*/

/*Search for temporary node with minimum distand make it current
node*/
min=infinity;
current=0;
for(i=1;i<=n;i++)
{
if(state[i].status == TEMP && state[i].dist < min)
{
min = state[i].dist;
current=i;
}
}/*End of for*/

if(current==0) /*If Source or Sink node is isolated*/
return 0;
state[current].status=PERM;
}/*End of while*/

/* Getting full path in array from destination to source */
while( current!=0 )
{
count++;
path[count]=current;
Serial.print(" <-- ");
//Serial.println(String(count));
//Serial.print(String(path[count]));
 switch(current){
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

current=state[current].predecessor;
if( current!=0 )
Serial.print(" ");
}

/*Getting distance from source to destination*/
for(i=count;i>1;i--)
{
u=path[i];
v=path[i-1];
*sdist+= adj[u][v];
}
return (count);
}/*End of findpath()*/



void setup()
{
  Serial2.begin(9600);

  Serial.begin(9600);
  Serial1.begin(9600);

  portal_beginning();
  getgps();
  while(1){
                   sendData( "AT+CGNSINF",1000);  //Sets GPS Mode
                   if(flag == 1) break;
           }
           getsource(current_lat.toFloat(),current_long.toFloat());

 
  if (!myDFPlayer.begin(Serial2)) {  //Use softwareSerial to
communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }

  myDFPlayer.volume(10);  //Set volume value (0~30).

  myDFPlayer.play(5);  //Play the first mp3
  delay(1000);
  
}

void loop()
{
  static unsigned long timer = millis();

  if (millis() - timer > 3000) {
    timer = millis();
    myDFPlayer.next();  //Play next mp3 every 3 second.
  }

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
  sendData( "AT+CGNSINF",1000); 
  delay(500);
  Serial.println("");
  findclosenode(current_lat.toFloat(),current_long.toFloat());
  nextnodeno = findnextnode(lastnode);
  Serial.print("\n Next Node is: ");
  Serial.println(nextnodeno);
  delay(200);
  Serial.print("\n Last Node Node is: ");
  Serial.println(lastnode);
  delay(200);
    if(nextnodeno != -1){
    next_lat = info[nextnodeno-1][1];
    next_long = info[nextnodeno-1][2];
   destbrng = angleFromCoordinate(current_lat.toFloat(),current_long.toFloat(),next_lat,next_long); //next_long   
    double distance = finddistance(current_lat.toFloat(),current_long.toFloat(),next_lat,next_long); //next_lat
    Serial.print("\n Distance from Next Node is ");
    Serial.println(distance);
    }
  else 
    {
    Serial.println("You have arrived at your destination");
    }
    //Serial.print("Sliced Time is : ");
    //Serial.println((current_time.substring(11,14)));
    //Serial.println(fmod((current_time.substring(11,14)).toFloat(),5));
 if(loopcount%3 == 0){  
  Serial.println("------------------------------------------------------------------");
  Serial.print("User's ");
  delay(500);
  usrbrng = angleFromCoordinate(lasttime_lat.toFloat(),lasttime_long.toFloat(),current_lat.toFloat(),current_long.toFloat()); //next_long   
   Serial.println("------------------------------------------------------------------");
  Serial.println(destbrng - usrbrng);
  lasttime_lat =  current_lat;
  lasttime_long = current_long;
  loopcount = 0;  
  }
  loopcount++;
  delay(100);
}
void getgps(void){
   sendData( "AT+CGNSPWR=1",1000); //Turn ON GPS Power Supply
   sendData( "AT+CGPSINF=0",1000); //Gets GPS Data
}

double finddistance(float lat1, float lon1, float lat2, float lon2){ 

  double earthRadiusKm = 6371;
  double dLat = (lat2-lat1)* 0.0174533;
  double dLon = (lon2-lon1)* 0.0174533;

  lat1 = lat1 * 0.0174533;
  lat2 = lat2 * 0.0174533;

  double a = pow(sin(dLat/2.0), 2) + cos(lat1) * cos(lat2) * pow(sin(dLon/2.0), 2); 
  double c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  return earthRadiusKm * c;
}

int findnextnode(int last){ 
  int i;
 
 // Serial.println(nodeno);
  for(i = nodeno ;i >= 2 ;i--)
    { 
      if(path[i]==last){
         return path[i-1];
      } 
    }  
     if(path[i]==last)  
         return -1;
  }
double angleFromCoordinate(float lat1, float long1, float lat2, float long2) {

    long int lati1, longi1, lati2, longi2;
    lati1 = lat1 * 1000000;
    
    longi1 = long1 * 1000000;
    
    lati2=  lat2 * 1000000;
    
    longi2 = long2 * 100000;
    Serial.print(" Coordinates 1 : "); delay(100);
    Serial.print((lati1)); delay(100);
    Serial.print(","); delay(100);
    Serial.println(String(longi1)); delay(100);
   
    Serial.print(" Coordinates 2 : "); delay(100);
    Serial.print(String(lati2)); delay(100);
    Serial.print(","); delay(100);
    Serial.println(String(longi2)); delay(100);
    
    double dLon = (long2 - long1);

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1)* cos(lat2) * cos(dLon);

    double brng = atan2(y, x);

    brng = brng * 57296 / 1000;
    
    String side = "Right";
    
    brng = (brng + 360);//
    brng = fmod(brng,360);
    brng = 360 - brng; // count degrees counter-clockwise - remove to make clockwise
    //Serial.println("");
    if(brng >180){
      brng = 360 - brng;
      side = "left";
     }
    Serial.print(" Bearing Angle is :");
    Serial.print(brng);
    Serial.print(" to the ");
    Serial.println(side);
    return brng;  
}
void getsource(float latitude, float longitude)
{
  int i,j;
  int source,dest;

  int shortdist,count;
  float lat,lon;
  source = findnodeno(latitude, longitude);

  lastnode = source;
 lasttime_lat = current_lat;  
 lasttime_long = current_long;
Serial.print("Your Current location is ");
switch(source){
   case 12  : Serial.println(" Campus Entry Gate"); break;
   delay(400);
   case 11 : Serial.println(" Admin Block"); break;
   delay(400);
   case 8 : Serial.println(" Girls Hostel"); break;
   delay(400);
   case 3 : Serial.println(" Library"); break;
   delay(400);
   case 1 : Serial.println(" A block"); break;
   delay(400);
   case 2 : Serial.println(" B block"); break;
   delay(400);
   case 4 : Serial.println(" C block"); break;
   delay(400);
   case 5 : Serial.println(" D block"); break;
   delay(400);
   case 9 : Serial.println(" Indian Bank"); break;
   delay(400);
   case 7 : Serial.println(" Canteen"); break;
   delay(400);
   case 10 : Serial.println(" Boys Hostel"); break;
   delay(400);
   case 6 : Serial.println(" E block"); break;
   delay(400);
 }


dest = portal_menu();
if(source==0 || dest==0)
{Serial.println("Invalid Node");
}

count = findpath(source,dest,&shortdist);
nodeno = count;
/*End of while*/
}

void sendData(String command,int timeout)
{
    String response = "";
    Serial1.println(command);
    delay(5);
    long int time = millis();
    while( (time+timeout) > millis()){
      while(Serial1.available()){
        response += char(Serial1.read());
      }
    }
  Serial.println(response);
      if(response.length()>=118  && flag == 0 || flag == 2)
      {
        int comma[10];
        comma[0] = response.indexOf(',');
        for(int i=1;i<6;i++){
             comma[i] = response.indexOf(',', comma[i-1] + 1 );
         }
      delay(100);
      current_time = response.substring(comma[1]+1,comma[2]);
      current_lat = response.substring(comma[2]+1 ,comma[3]);
      current_long = response.substring(comma[3]+1,comma[4]);
      current_alt = response.substring(comma[4]+1,comma[5]);
      
      /*Serial.print("current_time is : ");
      Serial.println(current_time);
       Serial.print("Your Latitude is : ");
      Serial.println(current_lat);
     Serial.print("Your Longitude is : ");
     Serial.println(current_long);
      Serial.print("Your Altitude is : ");
      Serial.println(current_alt);*/
    if (flag == 0) flag = 1;
    //else if(flag = 2) flag = 2;
     response = "";
      }
      else

      {
        flag =0;
        //Serial.println("Getting First time GPS data...");
       // response = "";
       }
}


void portal_beginning(void)
{

  Serial.flush();
//  Serial.println("******************************************************************************");
  Serial.println("                             G.G.S.I.P.U. BLIND PEDESTRIAN NAVIGATION SYSTEM");
//  Serial.println("******************************************************************************");
delay(1000);
    Serial.println("Please WAIT while we Fetch your Current Location");
}

int portal_menu(void)
{

 Serial.flush();
   delay(300);
      Serial.println("Choose Your Destination:-");

      delay(300);
     
   Serial.println(" 1. A block");
   delay(400);
   Serial.println(" 2. B block");
   delay(400);
   Serial.println(" 3. Library");
   delay(400);
   Serial.println(" 4. C block");
   delay(400);
   Serial.println(" 5. D block"); 
   delay(400);
   Serial.println(" 6. E block"); 
   delay(400);
   Serial.println(" 7. Canteen"); 
   delay(400);
   Serial.println(" 8. Girls Hostel"); 
   delay(400);
   Serial.println(" 9. Indian Bank"); 
   delay(400);
   Serial.println(" 10.  Boys Hostel"); 
   delay(400);
   Serial.println(" 11. Admin Block"); 
   delay(400);
   
   Serial.println(" 12. Campus Entry Gate"); 
   delay(400);
   Serial.print("Your Selected Destination is : ");

     while(Serial.available() == 0) { }// waiting for input
     String option = "6";


int option1 = option.toInt();
Serial.println(option1);
 switch(option1){
   case 1  : Serial.println("A block"); break;
   delay(400);
   case 2 : Serial.println(" B block"); break;
   delay(400);
   case 7 : Serial.println(" Canteen"); break;
   delay(400);
   case 3 : Serial.println(" Library"); break;
   delay(400);
   case 4 : Serial.println(" C block"); break;
   delay(400);
   case 5 : Serial.println(" D block"); break;
   delay(400);
   case 6 : Serial.println(" E block"); break;
   delay(400);
   case 8 : Serial.println(" Girls Hostel"); break;
   delay(400);
   case 12 : Serial.println(" Campus Entry Gate"); break;
   delay(400);
   case 9 : Serial.println("Indian Bank"); break;
   delay(400);
   case 10 : Serial.println(" Boys Hostel"); break;
   delay(400);
   case 11 : Serial.println("Admin Block"); break;
   delay(400);

    }
  return (option1);
}

void printDetail(uint8_t type, int value){
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
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
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




  


