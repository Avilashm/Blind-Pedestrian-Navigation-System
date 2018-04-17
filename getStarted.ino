int numarray[6] = { -1, -1, -1, -1, -1, -1}; //checksum -(36 71 78 82 77 67) = -411
int i = 0;
String response = "";
String toprint = "";
int comma[10];
byte gps;
int  sum = 0;
double lat_deg, long_deg,lat_min, long_min;
String current_lat;
String current_long;
String current_alt ;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);

  while (Serial2.available() > 0) {
    gps = Serial2.read();

    while (numarray[5] < 0)
    {

      numarray[i] = int(gps);
      char a = numarray[i];
      toprint = toprint + a;
      Serial.print(numarray[i]);
      i++;
    }
  }

}

void loop() {
  
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
    //Serial.println(response);
    //Serial.println(response.length());
    if(response.length()>=55)
      {
        comma[0] = response.indexOf(',');
        for(int i=1;i<6;i++){
             comma[i] = response.indexOf(',', comma[i-1] + 1 );
         }
      delay(100);
      current_lat = response.substring(comma[2]+1 ,comma[3]);
      current_long = response.substring(comma[4]+1,comma[5]);
     
      
      
       lat_deg = current_lat.substring(0,3).toFloat();
       lat_min = current_lat.substring(3).toFloat(); 
       
       long_deg = current_long.substring(0,3).toFloat();
       long_min = current_lat.substring(3).toFloat();

    lat_deg += lat_min/60;    
    long_deg += long_min/60;

    Serial.print("Latitiude");
    Serial.print(lat_deg* 10000);
    Serial.print("  Longitude");
    Serial.print(long_deg*10000); 
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
