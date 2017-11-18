#include <SoftwareSerial.h>
SoftwareSerial mySerial(7,8); 

struct vertices
{
    String lati,longi,named;
    struct edge *edgelist; 
   
}; //gate,lib,admin,a,b,c,d,e,cant,bh,gh;


struct edge
{
    struct vertices v1;  // pointer to head node of edge
    struct vertices v2;  // pointer to head node of edge
    int weight; 
    String info; 
}; 



struct Graph
{   

    int V; // Graph's no. of vertices
    struct vertices * Array;
};

struct vertices* newvertices(String lat, String lon, String nam)
{
    struct vertices* newV = (struct vertices*) malloc(sizeof(struct vertices));
    newV->lati = lat;
    newV->longi = lon;
    newV->named = nam;
    return newV;
}
 
struct Graph* createGraph(int V)
{
    struct Graph* graph = (struct Graph*) malloc(sizeof(struct Graph));
    graph->V = V;
 
    
    graph->Array = (struct vertices*) malloc(V * sizeof(struct vertices));
 
    int i;
    for (i = 0; i < V; ++i)
        graph->Array[i].lati = "0";
        graph->Array[i].longi = "0";
        graph->Array[i].named = "0";
      
    return graph;
}
 
// Adds an edge to an undirected graph
void addEdge(struct Graph* graph,struct vertices source, struct vertices destin)
{
   struct edge* newE = (struct edge*) malloc(sizeof(struct edge));
    newE->v1 = source;
    newE->v2 = destin;
    newE->info = "0";
    newE->weight = 1; 
}    


void setup(){
  Serial.begin(9600);
  mySerial.begin(9600); 
  getgps();
}

void loop(){
   sendData( "AT+CGNSOUT",1000);   //Sets GPS Mode 
}

void getgps(void){
   sendData( "AT+CGNSPWR=1",1000); //Turn ON GPS Power Supply 
   sendData( "AT+CGPSOUT=2",1000); //Gets GPS Data
}

void sendData(String command,int timeout)
{
    String response = "";    
    mySerial.println(command); 
    delay(5);
    if(1){
    long int time = millis();   
    while( (time+timeout) > millis()){
      while(mySerial.available()){       
        response += char(mySerial.read());
      }  
    }    
      Serial.println(response);
     /* if(response.length()>=118)
      {
        int comma[10];
        comma[0] = response.indexOf(',');
        for(int i=1;i<5;i++){
             comma[i] = response.indexOf(',', comma[i-1] + 1 );
         }
      delay(100);
     String current_lat = response.substring(comma[2]+1 ,comma[3]);
     String current_long = response.substring(comma[3]+1,comma[4]);
     Serial.println(current_lat);
     Serial.println(current_long);
     
     
    
      } */
    }  

      
}
