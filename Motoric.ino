bool berhentiPerulangan = false;
bool mulaiPerulangan = false;

void dapatkanBola(int pwm[]) {
  if (kondisi == 1 || kondisi == -1) {
    int  _pwm = (pwm[0] + pwm[1]) / 2;
    if (_pwm > 200) _pwm = 200;
    if (pwm[1] > 100) pwm[1] = 100;

    if (kondisi == 1) {
      berhentiPerulangan = true;
      if (mulaiPerulangan && !berhentiPerulangan) {
        for (int i = 0; i <= _pwm; i++) {
          if (i >= _pwm) mulaiPerulangan = false;
          analogWrite(pinL[0], i / 3);
          analogWrite(pinL[2], i);
          delay(10);
        }
      }
      analogWrite(pinL[0], _pwm / 3);
      analogWrite(pinL[2], _pwm);
      analogWrite(pinL[1], pwm[1]);
      analogWrite(pinL[3], pwm[1]);
    } else if (kondisi == -1 ) {
      berhentiPerulangan = false;
      if (!mulaiPerulangan && berhentiPerulangan) {
        for (int i = 0; i <= _pwm; i++) {
          if (i >= _pwm) mulaiPerulangan = true;
          analogWrite(pinL[0], i );
          analogWrite(pinL[2], i / 3);
          delay(10);
        }
      }
      analogWrite(pinL[0], _pwm);
      analogWrite(pinL[2], _pwm / 3);
      analogWrite(pinL[1], pwm[1]);
      analogWrite(pinL[3], pwm[1]);
    }


    analogWrite(pinR[0], 0);
    analogWrite(pinR[1], 0);
    analogWrite(pinR[2], 0);
    analogWrite(pinR[3], 0);
  } else {
    if (kondisi == 2) {
      putarKanan(pwm[0] + pwm[1]);
    } else if (kondisi == -2) {
      putarKiri(pwm[0] + pwm[1]);
    }
  }
}
void putarKanan(int pwm) {
  if (pwm >= 150) pwm = 150;
  if (!berhentiPerulangan) {
    for (int i = pwm; i >= 0; i--) {
      if (kondisi != 2 || i <= 0) {
        berhentiPerulangan = true;
        break;
      }
      analogWrite(pinL[2], i);
      analogWrite(pinL[3], i);
      delay(10);
    }
  }
  analogWrite(pinL[2], 0);
  analogWrite(pinL[3], 0);
  analogWrite(pinR[3], 0);
  analogWrite(pinR[2], 0);
}

void putarKiri(int pwm) {
  if (pwm >= 150) pwm = 150;
  if (berhentiPerulangan) {
    for (int i = pwm; i >= 0; i--) {
      if (kondisi != -2 || i <= 0) {
      berhentiPerulangan = false;
        break;
      }
      analogWrite(pinL[0], i);
      analogWrite(pinL[1], i);
      delay(10);
    }
  }
  analogWrite(pinL[0], 0);
  analogWrite(pinL[1], 0);
  analogWrite(pinR[1], 0);
  analogWrite(pinR[0], 0);
}

void serongKiri(int pwm) {
  analogWrite(pinL[2], pwm);
  analogWrite(pinR[2], 0);

  analogWrite(pinL[1], pwm);
  analogWrite(pinR[1], 0);

  analogWrite(pinL[3], 0);
  analogWrite(pinR[3], 0);
  analogWrite(pinR[0], 0);
  analogWrite(pinL[0], 0);
}

void serongKanan(int pwm) {
  analogWrite(pinL[0], pwm);
  analogWrite(pinR[0], 0);

  analogWrite(pinL[3], pwm);
  analogWrite(pinR[3], 0);

  analogWrite(pinL[2], 0);
  analogWrite(pinR[2], 0);
  analogWrite(pinR[1], 0);
  analogWrite(pinL[1], 0);
}
void maju(int pwm) {
  for (int i = 0; i < 4; i++) {
    analogWrite(pinL[i], pwm);
    analogWrite(pinR[i], 0);
  }
}

