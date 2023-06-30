//--------------------------------------LIBRERIAS--------------------------------------------------------
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
//Se incluyen las librerias necesarias para trabajar con los 2 sensores
#include <MQUnifiedsensor.h>
//Libreria definida desde EDGUE IMPULSE
#include <air_quality_inferencing.h>
//--------------------------------------------------------------------------------------------------------

//--------------------------------------Parametros libreria MQ--------------------------------------------
#define placa "ESP-32"
#define Voltage_Resolution 5
#define pin 15 //Pin analogico de la ESP
#define type "MQ-135" //Modelo MQ 
#define ADC_Bit_Resolution 12 // For ESP32
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
//---------------------------------------------------------------------------------------------------------

//---------------------------Parametros para el Wifi y bot de telegram-------------------------------------
const char* ssid = "ManuelMN10";        // Nombre de tu red Wi-Fi
const char* password = "12345678";   // Contraseña de tu red Wi-Fi

const char* telegramChatId = "1306137817";    // Token de acceso del bot de Telegram
const char* telegramToken = "6192139911:AAHL7C0w4iFMyG4ysx3qtSDD7-UtiZrLdfY";  // ID del chat de Telegram

WiFiClientSecure client;
UniversalTelegramBot bot(telegramToken, client);
//----------------------------------------------------------------------------------------------------------

//----------------------------Parametros para la predicción-------------------------------------------------
float CO, Alcohol, CO2, Toluen, NH4, Aceton;
float features[] = {
    CO, Alcohol, CO2, Toluen, NH4, Aceton
};

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void print_inference_result(ei_impulse_result_t result);
//-----------------------------------------------------------------------------------------------------------


//----------------------------------------------------SETUP----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  //-------------------------------------Calibracion MQ135---------------------------------------------------
  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init(); 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}

  Serial.println("Sensor Calibrado correctamente!!");
  //---------------------------------------------------------------------------------------------------------

  //-------------------------------Conexión Wi-Fi------------------------------------------------------------
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a la red Wi-Fi...");
  }
  Serial.println("Conectado a la red Wi-Fi");

  // Inicializar conexión segura para Telegram
  client.setInsecure();
  //---------------------------------------------------------------------------------------------------------
  
  delay(1000);
}
//-------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------LOOP-----------------------------------------------------------------------------
void loop() {
  //---------------------------------------------PARAMETROS MQ135---------------------------------------------
   MQ135.update(); // Update data, the arduino will read the voltage from the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
  CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  features[0] = CO; 

  MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
  Alcohol = MQ135.readSensor(); // SSensor will read PPM concentration using the model, a and b values set previously or from the setup
  features[1] = Alcohol; 

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  CO2 = 400 + MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  features[2] = CO2; 

  MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
  Toluen = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  features[3] = Toluen; 
  
  MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
  NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  features[4] = NH4; 

  MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
  Aceton = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  features[5] = Aceton; 
  //----------------------------------------------------------------------------------------------------------
  Serial.print("Datos recolectados: ");
  Serial.print(CO); 
  Serial.print(","); Serial.print(Alcohol); 
  Serial.print(","); Serial.print(CO2); 
  Serial.print(","); Serial.print(Toluen); 
  Serial.print(","); Serial.print(NH4); 
  Serial.print(","); Serial.println(Aceton);
  
  //----------------------------------------------Prediccion--------------------------------------------------
  ei_printf("Edge Impulse standalone inferencing (Arduino)\n");

    if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
        delay(1000);
        return;
    }

    ei_impulse_result_t result = { 0 };

    // the features are stored into flash, and we don't want to load everything into RAM
    signal_t features_signal;
    features_signal.total_length = sizeof(features) / sizeof(features[0]);
    features_signal.get_data = &raw_feature_get_data;

    // invoke the impulse
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }

    // print inference return code
    ei_printf("run_classifier returned: %d\r\n", res);
    print_inference_result(result);
  //----------------------------------------------------------------------------------------------------------
  //----------------------------------------------MENSAJE TELEGRAM--------------------------------------------
  String message = "Mensaje de prueba";
  bot.sendMessage(telegramChatId, message);
  Serial.println("Mensaje Enviado! Medicion tomada en ESP32");
  //-----------------------------------------------------------------------------------------------------------

  
  delay(5000);
}
//-------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------FUNCIONES---------------------------------------------------------------------------

void print_inference_result(ei_impulse_result_t result) {

    // Print how long it took to perform inference
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

    // Print the prediction results (object detection)
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

}
//---------------------------------------------------------------------------------------------------------------------------------
