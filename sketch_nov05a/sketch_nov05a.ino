
#include <SoftwareSerial.h>
SoftwareSerial mySerial(7,8); 

#define MAX 15
#define TEMP 0
#define PERM 1
#define infinity 999

 const float info[15][15] PROGMEM= {{1,28.594874,77.021519},
                       {2,28.594856,77.020942},
                       {3,28.594892,77.021355},
                       {4,28.594211,77.020073},
                      {5,28.594216,77.019617},
                      {6,28.594921,77.019724},
                      {7,28.594769,77.019083},
                      {8,28.595135,77.019553},
                      {9,28.595028,77.018457},
                      {10,28.594228,77.017949},
                      {11,28.594906,77.017949},
                      
                      }; 
            
         
int n = 11;

 int adj[MAX][MAX] =  {
   
    {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    {  0, 0, 2,  1, 6, 9, 6, 7, 2, 3, 4, 7},
    {  0, 1, 0,  3, 7, 9, 6, 7, 2, 3, 4, 7},
    {  0, 1, 52,  0, 19, 9, 6, 7, 2, 3, 4, 7},
    {  0, 52, 2,  1, 0, 9, 6, 7, 29, 3, 4, 7},
    {  0, 8, 2,  3, 2, 0, 6, 7, 2, 3, 4, 7},
    {  0, 7, 2,  4, 10,97, 0, 7, 27, 3, 4 ,7},
    {  0, 94, 72,  7, 7, 9, 6, 0, 2, 3, 4, 7},
    {  0, 0, 2,  8, 55, 9, 6, 7, 0, 30, 4, 7},
    {  0, 9, 82, 10, 6, 9, 6, 7, 2, 0, 4, 77},
    {  0, 10, 2,  6, 1, 9, 6, 7, 2, 3, 0, 7},
     { 0, 11, 2, 13, 7, 9, 6, 7, 2, 3, 4, 0},
    
};

  String current_lat ;
  String current_long ;
  String current_alt ;
     
struct node
{
int predecessor;
int dist; /*minimum distance of node from source*/
int status;

};

int findnodeno(float lat1, float lon1)
{ 

int k;
   double minsum = 9999,ans;
   int imin; 
   double latsq,longsq;
   info[4][1];
   for(k=0;k<n;k++)
     {
         latsq =  fabs(lat1 - (info[k][1])); 
        // printf("%f\t",latsq);
                    
         longsq = fabs(lon1 - (info[k][2])); 
        // printf("%f\t",longsq);
                      
         ans = longsq + latsq; 

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
Serial.print("   ");
Serial.print(adj[i][j]);
Serial.print("   ");
}
Serial.println("");
}

}



int findpath(int s,int d,int path[MAX],int *sdist)
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
//Serial.println(String(count));
Serial.print(String(path[count]));

current=state[current].predecessor;
if( current!=0 )
Serial.print("< - - ");
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


void setup(){
  Serial.begin(9600);
  mySerial.begin(9600); 
  portal_beginning();
  getgps();
  while(1){
                   sendData( "AT+CGNSINF",1000);  //Sets GPS Mode 
                   if(flag == 1) break;
           }   
           getsource(current_lat.toFloat(),current_long.toFloat());
         
}

void loop(){
  
  
}

void getgps(void){
   sendData( "AT+CGNSPWR=1",1000); //Turn ON GPS Power Supply 
   sendData( "AT+CGPSINF=0",1000); //Gets GPS Data
}

void getsource(float latitude, float longitude)
{
  int i,j;
  int source,dest;
  int path[MAX];
  int shortdist,count;
  float lat,lon;
//create_graph();
Serial.println("The adjacency matrix is : ");
display();

source = findnodeno(latitude, longitude);


Serial.println(source);

dest = portal_menu();
if(source==0 || dest==0)
{Serial.println("Invalid Node");
}

count = findpath(source,dest,path[MAX],&shortdist);
/*End of while*/
}
  
void sendData(String command,int timeout)
{
    String response = "";    
    mySerial.println(command); 
    delay(5);
    long int time = millis();   
    while( (time+timeout) > millis()){
      while(mySerial.available()){       
        response += char(mySerial.read()); 
      }  
    }    
     Serial.println(response);
      if(response.length()>=118  && flag == 0)
      {
        int comma[10];
        comma[0] = response.indexOf(',');
        for(int i=1;i<6;i++){
             comma[i] = response.indexOf(',', comma[i-1] + 1 );
         }
      delay(100);
      current_lat = response.substring(comma[2]+1 ,comma[3]);
      current_long = response.substring(comma[3]+1,comma[4]);
      current_alt = response.substring(comma[4]+1,comma[5]);
     Serial.print("Your Latitude is : ");
      Serial.println(current_lat);
     Serial.print("Your Longitude is : ");
     Serial.println(current_long);
      Serial.print("Your Altitude is : ");
     Serial.println(current_alt);
     flag = 1;
     
      }
      else    

      { 
        flag =0;
        Serial.println("Getting First time GPS data...");}
      } 


void portal_beginning(void)
{
  
  Serial.flush();
  Serial.println("              ******************************************************************************");
  Serial.println("                                  GGSIPU'S BLIND PEDESTRIAN NAVIGATION SYSTEM");
  Serial.println("              ******************************************************************************");
 
    Serial.println("Please WAIT while we Fetch your Current Location");
}

int portal_menu(void)
{
 
  Serial.flush();
      Serial.println("Choose Your Destination:-");
      Serial.println("1) Campus Entry Gate");
      Serial.println("2) Admin Block");
      Serial.println("3) Girls Hostel");
      Serial.println("4) Library");
      Serial.println("5) A block ");
      Serial.println("6) B block");
      Serial.println("7) C block");
      Serial.println("8) D block");
      Serial.println("9) E block");
      Serial.println("10) Canteen");
      Serial.println("11) Boy Hostel ");
     
      Serial.flush();
    Serial.print("Your Selected Destination is : ");
  
      while(Serial.available() == 0) { }  
      String option = Serial.readStringUntil("\n");
     
int option1; 

option1 = option.toInt();
 switch(option1){   
   case 1  : Serial.println(" Campus Entry Gate"); break;
   case 2 : Serial.println(" Admin Block"); break;
   case 3 : Serial.println(" Girls Hostel"); break;
   case 4 : Serial.println(" Library"); break;
   case 5 : Serial.println(" A block"); break;
   case 6 : Serial.println(" B block"); break;
   case 7 : Serial.println(" C block"); break;
   case 8 : Serial.println(" D block"); break; 
   case 9 : Serial.println( " E block"); break;
   case 10 : Serial.println(" Canteen"); break;
   case 11 : Serial.println(" Boys Hostel"); break;
   
    }
  return (option1);
}


   
