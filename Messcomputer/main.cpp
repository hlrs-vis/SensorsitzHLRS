#include <iostream>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <string>
#include <sstream>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringSerial.h>
#include <chrono>

using namespace std;
using byte = unsigned char;

/********
 *  Klasse für die Kombination von vier Kraftmessdosen
 */
class Scale {
public:
	byte input[33];
	long time1;
	long  force1;
	long time2;
	long force2;
	long time3;
	long force3;
	long time4;
	long force4;
	bool convert = false;
	bool print = false;
	
};

/********
 * Klasse für einen Beschleunigungssensor
 */
class Accelerometer {
public:
	byte input[11];
	long timeStamp = (long) (input[1]) + ((long) (input[2] << 8)) + ((long) (input[3] << 16)) + ((long) (input[4] << 24));
	
	int16_t accelx;
	int16_t accely;
	int16_t accelz;
	int offsetX;
	int offsetY;
	int offsetZ;
	
	int gainX;
	int gainY;
	int gainZ;
	
	bool convert = false;
	bool print = false;
	
	Accelerometer(int gx, int gy, int gz, int ox, int oy, int oz){
		offsetX = ox;
		offsetY = oy;
		offsetZ = oz;
		
		gainX = gx;
		gainY = gy;
		gainZ = gz;
	}
	
	double accelerationX(){
		long calcX = (accelx - offsetX) / gainX;
		return calcX;
	}
	
	double accelerationY(){
		long calcY = (accely - offsetY) / gainY;
		return calcY;
	}
	
	double accelerationZ(){
		long calcZ = (accelz - offsetZ) / gainZ;
		return calcZ;
	}
};


/********
 * 
 */
Scale seatScale;
Scale sBackScale;
Accelerometer seatAccelerometer(132, 133, 127, 3, 3, 31);
Accelerometer headAccelerometer(130, 132, 131, 1, 4, 11);
bool cond = true;


/********
 * Serieller Thread, hier die zu erwartenden Ports eintragen
 */
void *serialConnect(void *arg)
{
	cout  << "SerialConnect thread started...\n";
	
	int  fd1;
	enum Type {neutral, seat, sback, seatAccel, headAccel};
	Type type = neutral;
	bool ttyUSB0 = true;
	byte forceBuffer[33];
	byte accBuffer[11];
	int byteCounter = 0;
	

	if((fd1=serialOpen("/dev/ttyUSB0",115200))<0){
		fprintf(stderr,"Unable to open serial device: %s\n",strerror(errno));
		ttyUSB0 = false;
		cout << "Retrying opening serial connection...\n";
	}
	
	
	if (!ttyUSB0){
		if((fd1=serialOpen("/dev/ttyUSB1",115200))<0){
		fprintf(stderr,"Unable to open serial device: %s\n",strerror(errno));
	} else { cout <<"Connected to /dev/ttyUSB1 \n";}
	}
	
	cout << "SerialConnect: Trigger Senden"; 
	serialPutchar(fd1, 'k');
	
	while (cond == true){
		
		//cout << "Incoming data...\n";
		unsigned char incoming = serialGetchar(fd1);
		
		switch (type){
			case neutral: {
				switch (incoming){
					case 0: {
						type = seat;
						forceBuffer[0] = incoming;
						++byteCounter;
						break;
					}
					case 1: {
						type = sback;
						forceBuffer[0] = incoming;
						++byteCounter;
						break;
					}
					case 2: {
						type = seatAccel;
						accBuffer[0] = incoming;
						++byteCounter;
						break;
					}
					case 3: {
						type = headAccel;
						accBuffer[0] = incoming;
						++byteCounter;
						break;
					}
					default: type = neutral;
				}
				break;
			}
			
			case seat: {
				
				if (byteCounter < 32){
					forceBuffer[byteCounter] = incoming;
					++byteCounter;
					break;
				}
				
				if (byteCounter == 32){
					forceBuffer[byteCounter] = incoming;
					for (int i = 0; i < 33; ++i){
						seatScale.input[i] = forceBuffer[i];
					}

					seatScale.convert = true;
					byteCounter = 0;
					type = neutral;
					break;
				}
				break;
			}
			
			case sback: {
				if (byteCounter < 32){
					forceBuffer[byteCounter] = incoming;
					++byteCounter;
					break;
				}
				
				if (byteCounter == 32){
					forceBuffer[byteCounter] = incoming;
					for (int i = 0; i < 33; ++i){
						sBackScale.input[i] = forceBuffer[i];
					}
					sBackScale.convert = true;
					byteCounter = 0;
					type = neutral;
					break;
				}
				
				break;
			}
			
			case seatAccel:{
				if (byteCounter < 10){
					accBuffer[byteCounter] = incoming;
					++byteCounter;
					break;
				}
				
				if (byteCounter == 10){
					accBuffer[byteCounter] = incoming;
					
					for (int i = 0; i < 11; ++i){
						seatAccelerometer.input[i] = accBuffer[i];
					}
					
					seatAccelerometer.convert = true;
					byteCounter = 0;
					type = neutral;
					break;
				}
				
				break;
			}
			
			case headAccel: {
				if (byteCounter < 10){
					accBuffer[byteCounter] = incoming;
					++byteCounter;
					break;
				}
				
				if (byteCounter == 10){
					accBuffer[byteCounter] = incoming;
					
					for (int i = 0; i < 11; ++i){
						headAccelerometer.input[i] = accBuffer[i];
					}
					
					headAccelerometer.convert = true;
					byteCounter = 0;
					type = neutral;
					break;
				}
				
				break;
			}
			
			default: cout << "Invalid input";
		}
		
	}

	serialPutchar(fd1, 'k');
}


/******
 * Thread, um die Bytes aus dem Seriellen Port in  long und ints zu konvertieren
 */
 void *converter(void *arg)
 {
	 while (cond){
		 
		 if(seatScale.convert){
			seatScale.time1 = (long) (seatScale.input[1]) + ((long) (seatScale.input[2] << 8)) + ((long) (seatScale.input[3] << 16)) + ((long) (seatScale.input[4] << 24));
			seatScale.force1= (long) (seatScale.input[5]) + ((long) (seatScale.input[6] << 8)) + ((long) (seatScale.input[7] << 16)) + ((long) (seatScale.input[8] << 24));
			seatScale.time2 = (long) (seatScale.input[9]) + ((long) (seatScale.input[10] << 8)) + ((long) (seatScale.input[11] << 16)) + ((long) (seatScale.input[12] << 24));
			seatScale.force2 = (long) (seatScale.input[13]) + ((long) (seatScale.input[14] << 8)) + ((long) (seatScale.input[15] << 16)) + ((long) (seatScale.input[16] << 24));
			seatScale.time3 = (long) (seatScale.input[17]) + ((long) (seatScale.input[18] << 8)) + ((long) (seatScale.input[19] << 16)) + ((long) (seatScale.input[20] << 24));
			seatScale.force3 = (long) (seatScale.input[21]) + ((long) (seatScale.input[22] << 8)) + ((long) (seatScale.input[23] << 16)) + ((long) (seatScale.input[24] << 24));
			seatScale.time4 = (long) (seatScale.input[25]) + ((long) (seatScale.input[26] << 8)) + ((long) (seatScale.input[27] << 16)) + ((long) (seatScale.input[28] << 24));
			seatScale.force4 = (long) (seatScale.input[29]) + ((long) (seatScale.input[30] << 8)) + ((long) (seatScale.input[31] << 16)) + ((long) (seatScale.input[32] << 24));
			
			seatScale.convert = false;
			seatScale.print = true;
		 }
		 
		 if(sBackScale.convert){
			sBackScale.time1 = (long) (sBackScale.input[1]) + ((long) (sBackScale.input[2] << 8)) + ((long) (sBackScale.input[3] << 16)) + ((long) (sBackScale.input[4] << 24));
			sBackScale.force1= (long) (sBackScale.input[5]) + ((long) (sBackScale.input[6] << 8)) + ((long) (sBackScale.input[7] << 16)) + ((long) (sBackScale.input[8] << 24));
			sBackScale.time2 = (long) (sBackScale.input[9]) + ((long) (sBackScale.input[10] << 8)) + ((long) (sBackScale.input[11] << 16)) + ((long) (sBackScale.input[12] << 24));
			sBackScale.force2 = (long) (sBackScale.input[13]) + ((long) (sBackScale.input[14] << 8)) + ((long) (sBackScale.input[15] << 16)) + ((long) (sBackScale.input[16] << 24));
			sBackScale.time3 = (long) (sBackScale.input[17]) + ((long) (sBackScale.input[18] << 8)) + ((long) (sBackScale.input[19] << 16)) + ((long) (sBackScale.input[20] << 24));
			sBackScale.force3 = (long) (sBackScale.input[21]) + ((long) (sBackScale.input[22] << 8)) + ((long) (sBackScale.input[23] << 16)) + ((long) (sBackScale.input[24] << 24));
			sBackScale.time4 = (long) (sBackScale.input[25]) + ((long) (sBackScale.input[26] << 8)) + ((long) (sBackScale.input[27] << 16)) + ((long) (sBackScale.input[28] << 24));
			sBackScale.force4 = (long) (sBackScale.input[29]) + ((long) (sBackScale.input[30] << 8)) + ((long) (sBackScale.input[31] << 16)) + ((long) (sBackScale.input[32] << 24));
			
			sBackScale.convert = false;
			sBackScale.print = true;
		 }
		 
		 if(seatAccelerometer.convert){
			 seatAccelerometer.timeStamp = (long) (seatAccelerometer.input[1]) + ((long) (seatAccelerometer.input[2] << 8)) + ((long) (seatAccelerometer.input[3] << 16)) + ((long) (seatAccelerometer.input[4] << 24));
			 
			 seatAccelerometer.accelx = (int16_t)(seatAccelerometer.input[5]) + ((int16_t) (seatAccelerometer.input[6] << 8));
			 seatAccelerometer.accely = (int16_t)(seatAccelerometer.input[7]) + ((int16_t) (seatAccelerometer.input[8] << 8));
			 seatAccelerometer.accelz = (int16_t)(seatAccelerometer.input[9]) + ((int16_t) (seatAccelerometer.input[10] << 8));
			 
			 seatAccelerometer.convert = false;
			 seatAccelerometer.print = true;
		 }
		 
		 if(headAccelerometer.convert){
			 headAccelerometer.timeStamp = (long) (headAccelerometer.input[1]) + ((long) (headAccelerometer.input[2] << 8)) + ((long) (headAccelerometer.input[3] << 16)) + ((long) (headAccelerometer.input[4] << 24));
			 
			 headAccelerometer.accelx = (int16_t)(headAccelerometer.input[5]) + ((int16_t) (headAccelerometer.input[6] << 8));
			 headAccelerometer.accely = (int16_t)(headAccelerometer.input[7]) + ((int16_t) (headAccelerometer.input[8] << 8));
			 headAccelerometer.accelz = (int16_t)(headAccelerometer.input[9]) + ((int16_t) (headAccelerometer.input[10] << 8));
			 
			 headAccelerometer.convert = false;
			 headAccelerometer.print = true;
		 }
		 
	 }
 }


/********
 * 
 */
void *thread2(void *arg)
{
	cout << "Printing thread started...\n";
	
	bool run = true;
	int dataCounter = 0;
	FILE *log1;
	FILE *log2;
	FILE *log3;
	FILE *log4;
	
	cout <<"Printthread: Öffne Dateien \n";
	
	//Adresse der Logdateien festlegen
	log1 = fopen("/home/daniel/Dokumente/Logs/Sitz.csv", "w");
	log2 = fopen("/home/daniel/Dokumente/Logs/Lehne.csv", "w");
	log3 = fopen("/home/daniel/Dokumente/Logs/SitzBeschl.csv", "w");
	log4 = fopen("/home/daniel/Dokumente/Logs/KopfBeschl.csv", "w");
	
	cout << "Printthread: Schreibe dateiheader\n";
	fprintf(log1, "%s;%s;%s;%s;%s;%s;%s;%s;%s\n", "Millis 1","Kraft 1","Millis 2","Kraft 2", "Millis3", "Kraft 3", "Millis 4", "Kraft 4", "Summe");
	fprintf(log2, "%s;%s;%s;%s;%s;%s;%s;%s;%s\n", "Millis 1","Kraft 1","Millis 2","Kraft 2", "Millis3", "Kraft 3", "Millis 4", "Kraft 4", "Summe");
	fprintf(log3, "%s;%s;%s;%s;%s;%s;%s\n", "Millis","X","Y","Z","X[m/s^2]","Y[m/s^2]","Z[m/s^2]");
	fprintf(log4, "%s;%s;%s;%s;%s;%s;%s\n", "Millis","X","Y","Z","X[m/s^2]","Y[m/s^2]","Z[m/s^2]");
	
	
	cout << "Printthread: Warten auf Druckbefehl \n";
	auto startTime = chrono::high_resolution_clock::now();
	
	while(run){
		auto runningTime = chrono::high_resolution_clock::now() - startTime;
		auto sec = std::chrono::duration_cast<std::chrono::seconds>(runningTime).count();
		
		if  (seatScale.print){
			fprintf(log1, "\"%ld\";\"%ld\";\"%ld\";\"%ld\";\"%ld\";\"%ld\";\"%ld\";\"%ld\"\n", 
						seatScale.time1,seatScale.force1, seatScale.time2, seatScale.force2, seatScale.time3, seatScale.force3, seatScale.time4, seatScale.force4);
			seatScale.print = false;
			++dataCounter;
		}
		
		if (sBackScale.print){
			fprintf(log2, "\"%ld\";\"%ld\";\"%ld\";\"%ld\";\"%ld\";\"%ld\";\"%ld\";\"%ld\"\n", 
						sBackScale.time1,sBackScale.force1, sBackScale.time2, sBackScale.force2, sBackScale.time3, sBackScale.force3, sBackScale.time4, sBackScale.force4);
			sBackScale.print = false;
			++dataCounter;
		}
		
		if (seatAccelerometer.print){
			fprintf(log3, "\"%ld\";\"%d\";\"%d\";\"%ld\";\"%f\";\"%f\";\"%f\"\n", 
						seatAccelerometer.timeStamp, seatAccelerometer.accelx, seatAccelerometer.accely, seatAccelerometer.accelz, seatAccelerometer.accelerationX(), seatAccelerometer.accelerationY(), seatAccelerometer.accelerationZ());
			seatAccelerometer.print = false;
			++dataCounter;
		}
		
		if (headAccelerometer.print){
			fprintf(log4, "\"%ld\";\"%d\";\"%d\";\"%ld\";\"%f\";\"%f\";\"%f\"\n", 
						headAccelerometer.timeStamp, headAccelerometer.accelx, headAccelerometer.accely, headAccelerometer.accelz, headAccelerometer.accelerationX(), headAccelerometer.accelerationY(), headAccelerometer.accelerationZ());
			headAccelerometer.print = false;
			++dataCounter;
		}
		
		//Hier wird die Dauer der Messung spezifiziert
		if (sec > 150) {
			run = false;
			cond = false;
			cout << "Printthread: Full Datapoints";
		}
	}
	
	fclose(log1);
	fclose(log2);
	fclose(log3);
	fclose(log4);
	cout << "Printthread: Closed Logfiles";
}

int main()
{
        pthread_t my_thread1;
		pthread_t my_thread2;
		pthread_t my_thread3;
		
        int ret1;
		int ret2;
		int ret3;

        printf("In main: creating threads\n");
        ret1 =  pthread_create(&my_thread1, NULL, &serialConnect, NULL);
        if(ret1 != 0) {
                printf("Error: pthread_create() failed\n");
                exit(EXIT_FAILURE);
        }
		
        ret2 =  pthread_create(&my_thread2, NULL, &thread2, NULL);
        if(ret2 != 0) {
                printf("Error: pthread_create() failed\n");
                exit(EXIT_FAILURE);
		}
		
		ret3 = pthread_create(&my_thread3, NULL, &converter, NULL);
		if(ret3 != 0){
			printf("Error: pthread_create() failed\n");
			exit(EXIT_FAILURE);
		}

        pthread_exit(NULL);
}