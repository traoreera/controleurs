#include <Wire.h>
#include <math.h>

// Classe MPU6050
class MPU6050 {
  public:
    MPU6050(int addr) : addr(addr) {
      ACCEL_SCALE_FACTOR = 4096.0; // ±8g (32768 / 8g)
      GYRO_SCALE_FACTOR = 131.0; // ±250°/s (32768 / 250°/s)
      dt = 0.01; // Intervalle de temps (en secondes)
      num_calibration_samples = 2000; // Nombre d'échantillons pour la calibration
      gyro_offset[0] = gyro_offset[1] = gyro_offset[2] = 0.0;
    }

    void begin() {
      Wire.begin();
      configure();
      calibrateGyroscope();
    }

    void readInclination(float *angles) {
      float accel[3], gyro[3];
      readAcceleration(accel);
      readGyroscope(gyro);

      // Calcul des angles d'accélération
      float angle_pitch_acc = atan2(accel[1], sqrt(accel[0] * accel[0] + accel[2] * accel[2]));
      float angle_roll_acc = atan2(-accel[0], accel[2]);

      angle_pitch_acc *= RAD_TO_DEG;
      angle_roll_acc *= RAD_TO_DEG;

      // Filtrage de complémentarité pour l'angle pitch
      static float pitch = 0;
      pitch = 0.98 * (pitch + gyro[1] * dt) + 0.02 * angle_pitch_acc;

      // Filtrage de complémentarité pour l'angle roll
      static float roll = 0;
      roll = 0.98 * (roll + gyro[0] * dt) + 0.02 * angle_roll_acc;

      // Calcul de l'angle yaw à partir du gyroscope
      static float yaw = 0;
      yaw += gyro[2] * dt;

      angles[0] = pitch;
      angles[1] = roll;
      angles[2] = yaw;
    }

    void calibrateGyroscope() {
      // Attendre que le MPU soit stable
      delay(2000); // Attendre 2 secondes

      // Accumuler les lecturec:\Users\Hunters\Desktop\controleur\secure.hs du gyroscope pour la calibration
      float gyro_accum[3] = {0.0, 0.0, 0.0};
      float gyro_raw[3];
      for (int i = 0; i < num_calibration_samples; i++) {
        readGyroscope(gyro_raw);
        gyro_accum[0] += gyro_raw[0];
        gyro_accum[1] += gyro_raw[1];
        gyro_accum[2] += gyro_raw[2];
        delay(.1); // Attendre un court instant entre chaque lecture
        Serial.println(" samplates gyroscope is "+ String(i));
      }

      // Calculer la moyenne des lectures pour obtenir l'offset
      gyro_offset[0] = gyro_accum[0] / num_calibration_samples;
      gyro_offset[1] = gyro_accum[1] / num_calibration_samples;
      gyro_offset[2] = gyro_accum[2] / num_calibration_samples;

      Serial.print("Offset du gyroscope calibré: ");
      Serial.print(gyro_offset[0]);
      Serial.print(", ");
      Serial.print(gyro_offset[1]);
      Serial.print(", ");
      Serial.println(gyro_offset[2]);
    }

  private:
    int addr;
    float ACCEL_SCALE_FACTOR;
    float GYRO_SCALE_FACTOR;
    float dt;
    int num_calibration_samples;
    float gyro_offset[3];

    void writeRegister(byte reg, byte data) {
      Wire.beginTransmission(addr);
      Wire.write(reg);
      Wire.write(data);
      Wire.endTransmission();
    }

    void readRegister(byte reg, byte *buffer, int length) {
      Wire.beginTransmission(addr);
      Wire.write(reg);
      Wire.endTransmission();
      Wire.requestFrom(addr, length);
      for (int i = 0; i < length; i++) {
        buffer[i] = Wire.read();
      }
    }

    int16_t readWord2c(byte reg) {
      byte buffer[2];
      readRegister(reg, buffer, 2);
      int16_t value = (buffer[0] << 8) | buffer[1];
      if (value >= 0x8000) {
        value = -((65535 - value) + 1);
      }
      return value;
    }

    void configure() {
      // Configuration des registres du MPU6050
      writeRegister(0x6B, 0x00); // PWR_MGMT_1: Réveiller l'appareil
      writeRegister(0x1B, 0x00); // GYRO_CONFIG: ±250°/s
      writeRegister(0x1C, 0x10); // ACCEL_CONFIG: ±8g
      writeRegister(0x1A, 0x00); // CONFIG: DLPF
    }

    void readAcceleration(float *axes) {
      axes[0] = readWord2c(0x3B) / ACCEL_SCALE_FACTOR;
      axes[1] = readWord2c(0x3D) / ACCEL_SCALE_FACTOR;
      axes[2] = readWord2c(0x3F) / ACCEL_SCALE_FACTOR;
    }

    void readGyroscope(float *axes) {
      axes[0] = (readWord2c(0x43) - gyro_offset[0]) / GYRO_SCALE_FACTOR;
      axes[1] = (readWord2c(0x45) - gyro_offset[1]) / GYRO_SCALE_FACTOR;
      axes[2] = (readWord2c(0x47) - gyro_offset[2]) / GYRO_SCALE_FACTOR;
    }
};