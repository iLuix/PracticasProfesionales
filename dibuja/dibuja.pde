/*
  seguir camino
*/

int i;
int j;
int n;
String lines[];
PVector tar;
void setup() {
  size(600,600,P3D);
   lines = loadStrings("salida2.txt");
  println("there are " + lines.length + " lines");
  n=200;
  i=0;
  j=0;
  //  println(lines[i]);
  
  //hint(ENABLE_OPENGL_4X_SMOOTH);
  
}

void draw(){
  //background(100);
  String[] pos=split(lines[j],' ');
  if(i==0){
    n=int(pos[0]);
    tar=new PVector(float(pos[1]), float(pos[2]),float(pos[3]));
    j++;
    pos=split(lines[j],' ');
  }
  
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
  translate(float(pos[0]),600-1.0*float(pos[1]),-600+float(pos[2]));
  sphere(30);
  translate(0,-15,0);
  translate(15*float(pos[3]),-15*float(pos[4]),15*float(pos[5]));
  sphere(5);
  //ellipse(0,0,20,20);
  popMatrix();
  //fill(0,255,0);
  pushMatrix();
  translate(tar.x,600-1.0*tar.y,-600+tar.z);
  sphere(30);
  //ellipse(0,0,20,20);
  popMatrix();
  
  i++;
  j++;
  i%=n;
  j%=lines.length;
}
