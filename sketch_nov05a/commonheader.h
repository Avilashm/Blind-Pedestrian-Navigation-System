#define MAX 15
#define TEMP 0
#define PERM 1
#define infinity 999

String info[15][15]= {{"1","28.594874","77.021519"},
                       {"2","28.594856","77.020942"},
                       {"3","28.594892","77.021355"},
                       {"4","28.594211","77.020073"},
                      {"5","28.594216","77.019617"},
                      {"6","28.594921","77.019724"},
                      {"7","28.594769","77.019083"},
                      {"8","28.595135","77.019553"},
                      {"9","28.595028","77.018457"},
                      {"10","28.594228","77.017949"},
                      {"11","28.594906","77.017949"}};
            
 
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