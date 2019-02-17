void setup(){
    Serial.begin(9600);
    while(!Serial) {};
}
int a = 0;
char character;
int height = 0;
int velocity = 0;
void loop(){
  /*a += 1;
    //Serial.print("c_s");
    Serial.write(a);
    if(a > 179){
      a = 0;
    }
   */
  for(int i = 0; i < 10; i++){
      character = Serial.read();
      if(character == 'h'){
        String h_str="";
        for(int i = 0; i < 4; i++){
          character = Serial.read();
          if(character != 'v'){
            h_str.concat(character);
          }
          else{
            height = h_str.toInt();
            String v_str="";
            for(int i = 0; i < 5; i++){
              character = Serial.read();
              if(character != 'h'){
                v_str.concat(character);
              }
              else{
                velocity = v_str.toInt();
                break;
              }
            }
            break;
          }
        }
        break;
      }
  }
  Serial.print("Height:" );
  Serial.println(height);
  Serial.print("Velocity:" );
  Serial.println(velocity);
}
