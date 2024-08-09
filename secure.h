

#define Buzzer      4
#define ledTest     
float battery_voltage;

/*                          A1
                ______      |    ______
        vcc---|__r1__|------|---|__r2__|---gmd

        r1: 1.5K
        r2: 1K
*/


class SecureDrone{
  public:
    #define Batpin      A1
    void BatState(void){
      // On applique un simple filtre passe-bas pour filtrer le signal (Fc ≈ 10Hz et gain de ~2.5dB dans la bande passante)
      battery_voltage = battery_voltage * 0.92 + (analogRead(Batpin) + 65) * 0.09853;

      // si la batterie est comprit entre        8,0<bat<12,4 
      if (battery_voltage < 1240 && battery_voltage > 800){
        int coef= ((1240 - battery_voltage) / (float) 3500);
        pwm_L_F += pwm_L_F * coef;
        pwm_L_B += pwm_L_B * coef;
        pwm_R_F += pwm_R_F * coef;
        pwm_R_B += pwm_R_B * coef;
        }
      else{tone(Buzzer,1000);}
    }
};



