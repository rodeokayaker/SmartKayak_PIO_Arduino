#include "SmartPaddleBLE.h"

namespace SmartPaddleUUID {
    const char* SERVICE_UUID = "4b2de81d-c131-4636-8d56-83b83758f7ca";
    const char* FORCE_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    const char* IMU_UUID = "d2e5bfeb-d9f8-4b75-a295-d3f4032086ea";
    const char* STATUS_UUID = "667ACEB9-4E06-4325-B7FD-AE1FE82017D5";
    const char* SPECS_UUID = "8346E073-0CBB-4F34-B3B7-83203AC052DA";
    const char* ORIENTATION_UUID = "6EB39A41-1B23-4C63-92ED-B6236DE7E7A6";
    const char* BLADE_UUID = "C7D2019D-22C9-40C7-ABFB-28F570217153";
}

SmartPaddleBLE::~SmartPaddleBLE(){
    if(serial) {delete serial; serial=nullptr;}
    if(messageHandler) {delete messageHandler; messageHandler=nullptr;}
}



int SmartPaddleBLE::BLEMTU =247;// max({sizeof(IMUData), sizeof(loadData), sizeof(OrientationData),  sizeof(BladeData)})+4;





