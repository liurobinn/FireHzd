#include <Arduino.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <GPotential.h>

double MASS = 1; //in kg
double E16 = 27853;
double pitch;
double roll;
double yaw;

const int GROUND = 0;
const int LAUNCH = 1;
const int ASCENDING = 2;
const int PWRLSASCENDING= 3;
const int APOGEE = 4;
const int DESCENDING = 5;
int flightState = GROUND;


int16_t ax, ay, az;

double  ALTITUDE;//store the barometer value

class PID {
private:
        long double lastTime = millis()/10;
        long double integral = 0;
        long double lastErr = 0;

public:
        long double p;
        long double i;
        long double d;

        float UPDATE(long double err) {

                long double dt = (millis()/10 - lastTime);
                long double dx = err - lastErr;
                lastErr=err;
                integral += err*dt;
                lastTime = millis()/10;
                delay(5);

        return p*err + i*integral/100 + d*dx/(dt/100);
}
};

class THRUST_VECTOR_CONTROL {

private:
        double pos;

        double offsetX=4; //TVC Mount Offsets X
        double offsetY=0; //TVC Mount Offsets Y

        double XservoVal;
        double YservoVal;

        Servo X08_X;
        Servo X08_Y;

        PID xPID;
        PID yPID;

        int t;

public:
        

        double XUpperLimit= 99 + offsetX;
        double YUpperLimit= 81 + offsetX;
        double XLowerLimit= 75 + offsetY;
        double YLowerLimit= 105 + offsetY;

        void SERVO_INIT(){//servo initialization

                X08_X.attach(3);
                X08_Y.attach(4);

                X08_X.write(90+offsetX);
                X08_Y.write(90+offsetY);

                pos= 90 + offsetX;

        //rotation test
        for (t=0; t>= 0 && t <=1440; t++) {
                roll=asin(0.05*cos((3.14/180)*t));
                pitch=asin(0.07*sin((3.14/180)*t));

                X08_X.write(roll * 180+90+offsetX);
                X08_Y.write(pitch * 180+90+offsetY);
                delay(5);
        }

        X08_X.write(90+offsetX);
        X08_Y.write(90+offsetY);

        }

        void MOTOR_EJECTION(){ //needs some work done
        X08_X.write(90+offsetX);
        X08_Y.write(90+offsetY);
        for (pos=90+offsetY; pos >= 65 +offsetY; pos -= 1) {

                X08_Y.write(pos);
                delay(3);
        }
        delay(10);

        for (pos=65+offsetY; pos <= 90 +offsetY; pos += 1) {

                X08_Y.write(pos);
                delay(1);
        }
        }

        void ASCENDING(){
                xPID.p = 0.5;
                xPID.i = 0.001;
                xPID.d = 0.01;

                yPID.p = 0.5;
                yPID.i = 0.001;
                yPID.d = 0.01;

                XservoVal= xPID.UPDATE(roll-90)+90+offsetX;
                YservoVal= yPID.UPDATE(pitch-90)+90+offsetY;

                X08_X.write(XservoVal);        // real servo output
                X08_Y.write(YservoVal);        // real servo output
        }
        void DESCENDING(){
                xPID.p = 0.3;
                xPID.i = 0.001;
                xPID.d = 0.001;

                yPID.p = 0.3;
                yPID.i = 0.001;
                yPID.d = 0.001;

                //altitude processing
                //acceleration
                //thrust curve

                XservoVal= xPID.UPDATE(roll-90)+90+offsetX;
                YservoVal= yPID.UPDATE(pitch-90)+90+offsetY;

                X08_X.write(XservoVal);        // real servo output
                X08_Y.write(YservoVal);        // real servo output

        }
};

class PYRO {
private:
        bool MotorOne= false;
public:
        bool MotorTwo= false;
        void INIT(){
                pinMode(5,OUTPUT);//Pyro 1
                pinMode(6,OUTPUT);//Pyro 2
                pinMode(7,OUTPUT);//Pyro 3
                pinMode(8,OUTPUT);//Pyro 4
        }

        void ASCENDING_IGNITION(){
                digitalWrite(7,HIGH);
                delay(2000);
                digitalWrite(7,LOW);
                delay(2000);

                flightState=LAUNCH;
        }
        bool DSCENDING_IGNITION(){
                digitalWrite(8,HIGH);
                delay(2000);
                digitalWrite(8,LOW);
                delay(2000);

                MotorTwo=true;

                return MotorTwo;
        }
        void FLAP_DEPLOY(){
                digitalWrite(5,HIGH);
                delay(5000);
                digitalWrite(5,LOW);
                delay(10);
        }
};

class LED {
public:
        void INIT(){
                pinMode(14, OUTPUT);//R
                pinMode(15, OUTPUT);//G
                pinMode(16, OUTPUT);//B
        }
};

class BUZZER {
public:
        void INIT(){
                pinMode(10,OUTPUT);
        }
};

class BAROMETER {
private:
        Adafruit_BMP280 BMP280;
public:
double LAUNCHALTITUDE;
        void INIT(){

                if (!BMP280.begin()) {
                        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                                 "try a different address!"));

                        while (1) delay(10);
                }

                BMP280.setSampling(Adafruit_BMP280::MODE_NORMAL,        /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,         /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,         /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,         /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500);         /* Standby time. */

                        LAUNCHALTITUDE = BMP280.readAltitude(1013.25);

                }

        double UPDATE_TEMPERATURE(){

                //Serial.print(F("Temp:"));
                //Serial.print(BMP280.readTemperature());
                //Serial.print(" *C"); Serial.println("\t");
                return BMP280.readTemperature();
        }

        double UPDATE_PRESSURE(){

                //Serial.print(F("Pressure:"));
                //Serial.print(BMP280.readPressure());
                //Serial.print(" Pa"); Serial.print("\t");
                return BMP280.readPressure();
        }

        double UPDATE_ALTITUDE(){

                //Serial.print(F("Alt:"));
                //Serial.print(BMP280.readAltitude(1013.25));
                //Serial.print(" m"); Serial.println("\t");
                return BMP280.readAltitude(1013.25);
        }
};

volatile bool mpuInterrupt = false;

void dmpDataReady() {
mpuInterrupt = true;
}

class IMU {
private:
        #define OUTPUT_READABLE_WORLDACCEL
        #define OUTPUT_READABLE_YAWPITCHROLL
        #define INTERRUPT_PIN 2
        #define LED_PIN 13

        bool blinkState = false;
        bool dmpReady = false;

        uint8_t mpuIntStatus;
        uint8_t devStatus;
        uint16_t packetSize;
        uint16_t fifoCount;
        uint8_t fifoBuffer[64];

        // orientation/motion vars
        Quaternion q;
        VectorInt16 aa;
        VectorInt16 aaReal;
        VectorInt16 aaWorld;
        VectorFloat gravity;

        MPU6050 mpu;
        float euler[3];
        float ypr[3];
        double val=0;
        double prev;

public:
        double RWAcc;
        void INIT(){
                // join I2C bus (I2Cdev library doesn't do this automatically)
                #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
                Wire.begin();
                Wire.setClock(400000);         // 400kHz I2C clock. Comment this line if having compilation difficulties
                #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
          #endif
        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);

        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();

        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788);         // 1688 factory default for my test chip

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
                // Calibration Time: generate offsets and calibrate our MPU6050
                mpu.CalibrateAccel(6);
                mpu.CalibrateGyro(6);
                mpu.PrintActiveOffsets();
                // turn on the DMP, now that it's ready
                Serial.println(F("Enabling DMP..."));
                mpu.setDMPEnabled(true);

                // enable Arduino interrupt detection
                Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
                Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
                Serial.println(F(")..."));
                attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
                mpuIntStatus = mpu.getIntStatus();

                // set our DMP Ready flag so the main loop() function knows it's okay to use it
                Serial.println(F("DMP ready! Waiting for first interrupt..."));
                dmpReady = true;

                // get expected DMP packet size for later comparison
                packetSize = mpu.dmpGetFIFOPacketSize();
        }
        else {
                // ERROR!
                // 1 = initial memory load failed
                // 2 = DMP configuration updates failed
                // (if it's going to break, usually the code will be 1)
                Serial.print(F("DMP Initialization failed (code "));
                Serial.print(devStatus);
                Serial.println(F(")"));
        }

        // configure LED for output
        pinMode(LED_PIN, OUTPUT);
        }

        void UPDATE(){

                if (!dmpReady) return;

                // wait for MPU interrupt or extra packet(s) available
                while (!mpuInterrupt && fifoCount < packetSize) {
                        if (mpuInterrupt && fifoCount < packetSize) {
                        // try to get out of the infinite loop
                        fifoCount = mpu.getFIFOCount();
                        }
                }

                mpuInterrupt = false;
                mpuIntStatus = mpu.getIntStatus();
                fifoCount = mpu.getFIFOCount();

                if(fifoCount < packetSize) {
                        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
                        // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
                }
                // check for overflow (this should never happen unless our code is too inefficient)
                else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
                        // reset so we can continue cleanly
                        mpu.resetFIFO();
                        //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
                        Serial.println(F("FIFO overflow!"));

                        // otherwise, check for DMP data ready interrupt (this should happen frequently)
                } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

                        // read a packet from FIFO
                        while(fifoCount >= packetSize) {         // Lets catch up to NOW, someone is using the dreaded delay()!
                                mpu.getFIFOBytes(fifoBuffer, packetSize);
                                // track FIFO count here in case there is > 1 packet available
                                // (this lets us immediately read more without waiting for an interrupt)
                                fifoCount -= packetSize;
                        }

                #ifdef OUTPUT_READABLE_YAWPITCHROLL
                        // display Euler angles in degrees
                        mpu.dmpGetQuaternion(&q, fifoBuffer);
                        mpu.dmpGetGravity(&gravity, &q);
                        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                        //Serial.print("ypr\t");
                        //Serial.print(yaw);
                        //Serial.print("\t");

                        //Serial.print(abs(abs(ypr[1] * 180/M_PI)-90));Serial.print("\n");

                        if(ypr[1] * 180/M_PI > 0) {
                                pitch = 90+abs(abs(abs(ypr[1] * 180/M_PI)-90)-90);
                                //Serial.print(pitch);Serial.print("\n");
                        }
                        else{
                                pitch = 90-abs(abs(abs(ypr[1] * 180/M_PI)-90)-90);
                                //Serial.print(pitch);Serial.print("\n");
                        }
                        //Serial.print("\t");
                        roll= abs(ypr[2] * 180/M_PI);
                        yaw= ypr[0] * 180/M_PI;
                        //Serial.println(roll);
                #endif

                #ifdef OUTPUT_READABLE_WORLDACCEL
                // display initial world-frame acceleration, adjusted to remove gravity
                // and rotated based on known orientation from quaternion
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                RWAcc=sqrt(aaWorld.x*aaWorld.x+aaWorld.y*aaWorld.y+aaWorld.z*aaWorld.z);//stationary is around 0~50
                //Serial.println(RWAcc);
                /*
                Serial.print("aworld\t");
                Serial.print(aaWorld.x);
                Serial.print("\t");
                Serial.print(aaWorld.y);
                Serial.print("\t");
                Serial.println(aaWorld.z);
                */
                #endif


                        blinkState = !blinkState;
                        digitalWrite(LED_PIN, blinkState);
                }
        }

        void ACC_UPDATE(){
                mpu.getAcceleration(&ax, &ay, &az);
                /*
                       Serial.print(ax/16384.00); Serial.print("\t");
                       Serial.print(ay/16384.00); Serial.print("\t");
                       Serial.print(az/16384.00); Serial.print("\t");
                */
        }
};

double processTime = micros()/1000000.000;

class SD_CARD{

private:
        File myFile;
        const int chipSelect = BUILTIN_SDCARD;
        Adafruit_BMP280 bmp280;
public:
        void INIT(){
                if (!SD.begin(chipSelect)) {
                //Serial.println("error");
                return;
                }
                myFile = SD.open("TVC_test.csv", FILE_WRITE);
                myFile.print("Time (s)"); myFile.print("\t");

                myFile.print("Rotation X (deg)"); myFile.print("\t");
                myFile.print("Rotation Y (deg)"); myFile.print("\t");
                myFile.print("Rotation Z (deg)"); myFile.print("\t");

                myFile.print("Accel X (g)"); myFile.print("\t");
                myFile.print("Accel Y (g)"); myFile.print("\t");
                myFile.print("Accel Z (g)"); myFile.print("\t");

                myFile.print("Altitude (ft)"); myFile.print("\t");
                myFile.print("Temp (F)"); myFile.print("\t");
                myFile.print("Humidity"); myFile.print("\t");
                myFile.println("Pressure");
                myFile.close();          

}

void write(){
        myFile = SD.open("TVC_test.csv", FILE_WRITE);
        myFile.print(micros()/1000000.000-processTime); myFile.print("\t");
        myFile.print(roll-90); myFile.print("\t");
        myFile.print(pitch-90); myFile.print("\t");
        myFile.print(yaw); myFile.print("\t");

        myFile.print(-az/16384.00); myFile.print("\t");
        myFile.print(-ay/16384.00); myFile.print("\t");
        myFile.print(-ax/16384.00); myFile.print("\t");
        myFile.close();

}
};

class  COUNTDOWN {
public:
        void startIndicator(){
                digitalWrite(10,HIGH);
                delay(1000);
                digitalWrite(10,LOW);
                delay(1000);
                digitalWrite(10,HIGH);
                delay(1000);
                digitalWrite(10,LOW);
                delay(1000);
        }
        void countdown(){
                int i=1;
                int counter;
                for(counter=1; counter >0; counter--) {
                        for (i=1; i<=counter; i++) {
                                digitalWrite(10,HIGH);
                                digitalWrite(14,LOW);
                                delay(200);
                                digitalWrite(10,LOW);
                                digitalWrite(14,HIGH);
                                delay(200);
                        }
                        digitalWrite(10,LOW);
                        digitalWrite(14,HIGH);
                        delay(10000);
                }
                int finalCount;
                for (finalCount=0; finalCount <= 10; finalCount++){
                digitalWrite(10,HIGH);
                digitalWrite(14,LOW);
                delay(500);
                digitalWrite(10,LOW);
                digitalWrite(14,HIGH);
                delay(500);
              }
        }
};

class FlightCtrl {
private:
        THRUST_VECTOR_CONTROL TVC;
        IMU mpu6050;
        BAROMETER baro;
        PYRO pyro;
        GPotential Ep;
        BUZZER buzzer;
        LED led;
        COUNTDOWN countdown;
        bool block1 = true;
        bool block2 = true;
        bool block3 = true;
        bool block4 = true;
        bool block5 = true;
        bool block6 = true;
        

public:
double apogee;
double liftoffTime;
        void INIT(){
                mpu6050.INIT();
                Wire.begin();
                baro.INIT();
                TVC.SERVO_INIT();
                buzzer.INIT();
                led.INIT();
                //countdown.startIndicator();
                //countdown.countdown();
                pyro.ASCENDING_IGNITION();
                liftoffTime=micros()/1000000.000;
                
        }
        void MAIN() {

                mpu6050.UPDATE();
                mpu6050.ACC_UPDATE();

                if(flightState == LAUNCH && block3) {
                        flightState = ASCENDING;
                        block3 = false;
                        Serial.println("LAUNCH"); 
                }else if(flightState == ASCENDING && block4) {
                        TVC.ASCENDING();
                        Serial.println(mpu6050.RWAcc);
                        Serial.println("ASCENDING");
                        flightState=PWRLSASCENDING;
                        
                }else if(flightState == PWRLSASCENDING && block1){
                        Serial.println(mpu6050.RWAcc);
                        if(mpu6050.RWAcc<=150){
                                apogee = baro.UPDATE_ALTITUDE();
                                flightState=APOGEE;
                                block1 = false;
                        }
                        Serial.println("PWRLSASCENDING");
                }else if(flightState == APOGEE && block2) {
                        digitalWrite(10,HIGH);
                        delay(50);
                        digitalWrite(10,LOW);
                        delay(50);
                        TVC.MOTOR_EJECTION();
                        pyro.FLAP_DEPLOY();
                        digitalWrite(10,HIGH);
                        delay(50);
                        digitalWrite(10,LOW);
                        delay(50);
                        flightState = DESCENDING;
                        block2 = false;
                        Serial.println("APOGEE");
                }else if(flightState == DESCENDING){

                        if(Ep.GetPotentialEnergy(baro.UPDATE_ALTITUDE()-baro.LAUNCHALTITUDE, MASS) - E16 == 50, block6){//give of margin of error
                                pyro.DSCENDING_IGNITION();
                                block6 = false;
                                block5 = false;
                        }else if(Ep.GetPotentialEnergy(apogee, MASS)<E16,block5){
                                block5 = false;
                                pyro.DSCENDING_IGNITION();
                        }
                        //if the potential energy is below the energy that E16 produces, fire the motor immediately
                        TVC.DESCENDING();       
                        Serial.println("DESCENDING"); 
                }

        }
};


FlightCtrl FlightControl;
SD_CARD sd;

void setup(){
        Serial.begin(115200);
        FlightControl.INIT();
}

void loop() {
        if(micros()/1000000.000-FlightControl.liftoffTime<=4){
                flightState=ASCENDING;
        }
        FlightControl.MAIN();
        //sd.write();
}