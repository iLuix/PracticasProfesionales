/*
  miercoles 12pm
  
  **arribar
  merodear
  huirr
  seguir camino
*/

int i;
int n;
String lines[];
PVector tar;
void setup() {
  size(1500,1000);
   //lines = loadStrings("salida_follow_path.txt");
   //lines = loadStrings("salida2.txt");
   lines = loadStrings("salida_colisiones.txt");
  println("there are " + lines.length + " lines");
  n=int(lines[0]);
  i=0;
  draw_map();

      
  
}
void draw_map(){
   background(255);
   fill(100);
      beginShape();
    vertex(0, 475);
    vertex(500, 475);
    vertex(500, 500);
    vertex(0, 500);
    endShape();
    
    beginShape();
    vertex(750, 0);
    vertex(750, 475);
    vertex(550,475);
   vertex(550, 500);
    vertex(775, 500);
    vertex(775, 0);
    endShape();
    beginShape();
    vertex(825, 475);
    vertex(1500, 475);
    vertex(1500, 500);
    vertex(825, 500);
    endShape();
    beginShape();
    vertex(600, 1000);
    vertex(600, 600);
    vertex(200, 600);
    vertex(200, 575);
    vertex(1300, 575);
    vertex(1300, 600);
    vertex(625, 600);
    vertex(625, 1000);
    endShape();
    beginShape();
    vertex(150, 150);
    vertex(600, 150);
    vertex(600, 300);
    vertex(150, 300);
    endShape();
    beginShape();
    vertex(150,100);
    vertex(200,100);
    vertex(200,125);
    vertex(150,125);
    endShape();
    beginShape();
    vertex(250,100);
    vertex(300,100);
    vertex(300,125);
    vertex(250,125);
    endShape();
    beginShape();
    vertex(350,100);
    vertex(400,100);
    vertex(400,125);
    vertex(350,125);
    endShape();
    beginShape();
    vertex(450,100);
    vertex(500,100);
    vertex(500,125);
    vertex(450,125);
    endShape();
    beginShape();
    vertex(550,100);
    vertex(600,100);
    vertex(600,125);
    vertex(550,125);
    endShape();
    beginShape();
    vertex(150,325);
    vertex(200,325);
    vertex(200,350);
    vertex(150,350);
    endShape();
    beginShape();
    vertex(250,325);
    vertex(300,325);
    vertex(300,350);
    vertex(250,350);
    endShape();
    beginShape();
    vertex(350,325);
    vertex(400,325);
    vertex(400,350);
    vertex(350,350);
    endShape();
    beginShape();
    vertex(450,325);
    vertex(500,325);
    vertex(500,350);
    vertex(450,350);
    endShape();
    beginShape();
    vertex(550,325);
    vertex(600,325);
    vertex(600,350);
    vertex(550,350);
    endShape();
    beginShape();
    vertex(850,75);
    vertex(925,75);
    vertex(925,275);
    vertex(850,275);
    endShape();
    beginShape();
    vertex(975,75);
    vertex(1050,75);
    vertex(1050,275);
    vertex(975,275);
    endShape();
    beginShape();
    vertex(1100,75);
    vertex(1175,75);
    vertex(1175,275);
    vertex(1100,275);
    endShape();
    beginShape();
    
    vertex(950,350);
    vertex(1200,350);
    vertex(1200,425);
    vertex(950,425);
    endShape();
    beginShape();
    vertex(1300,475);
    vertex(1500,475);
    vertex(1500,200);
    vertex(1450,200);
    vertex(1450,350);
    vertex(1375,425);
    vertex(1300,425);
    endShape();
    beginShape();
    vertex(550, 700);
    vertex(550, 900);
    vertex(600, 900);
    vertex(600, 700);
    endShape();
    beginShape();
    vertex(300, 950);
    vertex(500, 950);
    vertex(500, 900);
    vertex(300, 900);
    endShape();
    beginShape();
    vertex(250, 875);
    vertex(325, 700);
    vertex(265, 675);
    vertex(190, 850);
    endShape();
    beginShape();
    vertex(400, 675);
    vertex(500, 675);
    vertex(500, 725);
    vertex(400, 725);
    endShape();
    beginShape();
    vertex(700, 650);
    vertex(750, 650);
    vertex(750, 900);
    vertex(700, 900);
    endShape();
    beginShape();
    vertex(800, 650);
    vertex(850, 650);
    vertex(850, 900);
    vertex(800, 900);
    endShape();
    beginShape();
    vertex(900, 650);
    vertex(950, 650);
    vertex(950, 900);
    vertex(900, 900);
    endShape();
    beginShape();
    vertex(1000, 650);
    vertex(1050, 650);
    vertex(1050, 900);
    vertex(1000, 900);
    endShape();
    beginShape();
    vertex(1100, 650);
    vertex(1150, 650);
    vertex(1150, 900);
    vertex(1100, 900);
    endShape();
    beginShape();
    vertex(1200, 650);
    vertex(1250, 650);
    vertex(1250, 900);
    vertex(1200, 900);
    endShape();
    beginShape();
    vertex(1475, 650);
    vertex(1500, 650);
    vertex(1500, 900);
    vertex(1475, 900);
    endShape();
  
}
void draw(){
  String[] pos=split(lines[i+1],' ');
  i++;
  if(i%n==0)
    //draw_map();
  i%=n;
  //background(255);
  noStroke();
  fill(0);
  
  
  /*
  beginShape();
vertex(x1,y1);
vertex(x2,y2);
vertex(x3,y3);
vertex(x4,y4);
endShape();*/
  
  float x=float(pos[0]);
  float y=float(pos[1]);
  int k=int(pos[2]);
  if(k==1){
     float x1=float(pos[3]),y1=float(pos[4]);
     line(x,y,x1,y1);
     fill(0,255,0);
     ellipse(x1,y1,5,5);
     
    fill(0,0,200);
    
    
    
  }
  else
    fill(255,0,0);
  ellipse(x,y,5,5);
  
}