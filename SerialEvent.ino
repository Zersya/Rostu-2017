
//-1 = serong Kanan
//1 = serong kiri
//-2 = putar Kanan
//2 = putar kiri
//0 = maju
int _kondisiData = 0;

int pwmRot = 0;
int pwmDst = 0;
int kondisi = 0;

bool check = false;

String _dataRead = "";

int pinL[] = {3, 5, 7, 8};
int pinR[] = {2, 4, 6, 9};


void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(pinL[i], OUTPUT);
    pinMode(pinR[i], OUTPUT);
  }
  Serial.begin(9600);

   if(!mag.begin())
  {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
}

void loop() {
  //maju(100);
  serialEvent();
  if (check) {
    pisah();
    int pwm[] = {pwmRot, pwmDst};
    dapatkanBola(pwm);
   

    check = false;
    _dataRead = "";
  }
}

void serialEvent() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    _dataRead += inChar;
    if (inChar == '\n') {
      check = true;
    }
  }
}

void pisah() {
  int j = 0;
  String dataParse[50];
  dataParse[j] = "";

  for (int i = 0; i < _dataRead.length(); i++) {
    if (_dataRead[i] == ',') {
      j++;
      dataParse[j] = "";
    } else {
      dataParse[j] += _dataRead[i];
    }
  }

  if (dataParse != NULL) {
    pwmRot = dataParse[0].toInt();
    pwmDst = dataParse[1].toInt();
    kondisi = dataParse[2].toInt();

    Serial.print(pwmRot);
    Serial.print(",");
    Serial.print(pwmDst);
    Serial.print(",");
    Serial.println(kondisi);
  }
}



