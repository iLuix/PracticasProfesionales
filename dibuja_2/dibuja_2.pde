/*
  miercoles 12pm
  
  **arribar
  merodear
  huir
  seguir camino
*/

int i;
int n;
String lines[];
PVector tar;
void setup() {
  size(600,600,P3D);
   lines = loadStrings("salida2.txt");
  println("there are " + lines.length + " lines");
  n=int(lines[0]);
  i=0;
}

void draw(){
  String[] pos=split(lines[i+1],' ');
  i++;
  i%=n;
  background(0);
  noStroke();
  pointLight(300, 300, -300, 35, 40, 255);
  pushMatrix();
  rotateX(-3.1415692/2);
  translate(0,0,600);
  rect(0,0,600,600);
  
  popMatrix();
  
  //fill(255,255,255);
  pushMatrix();
  float x=float(pos[0]);
  float y=float(pos[1]);
  float z=float(pos[2]);
  
  while(x>600) x-=600;
  while(x<0) x+=600;
  while(y>600) y-=600;
  while(y<0) y+=600;
  while(z>600) z-=600;
  while(z<0) z+=600;
  
  translate(x,600-1.0*y,-600+z);
  sphere(30);
  
  //ellipse(0,0,20,20);
  popMatrix();
  //fill(0,255,0);
  
}