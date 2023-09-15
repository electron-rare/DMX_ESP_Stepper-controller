#define nb_dmx_channel 2                  // nombre de canaux DMX utilisés
#define pos_array 0                       // position de la position dans le tableau DMX_data
#define speed_array 1                     // position de la vitesse dans le tableau DMX_data

int DMX_start_channel;                    // numéro de la première adresse DMX
int pos_channel;                          // numéro de l'adresse DMX de la position
int speed_channel;                        // numéro de l'adresse DMX de la vitesse
const dmx_port_t dmx_num = DMX_NUM_2;     // numéro du port DMX
dmx_config_t config = DMX_CONFIG_DEFAULT; // configuration du port DMX
byte data[DMX_PACKET_SIZE];               // tableau de données DMX

bool dmxIsConnected = false;              // état de la connexion DMX
unsigned long lastUpdate = millis();      // temps de la dernière mise à jour des données DMX
uint8_t DMX_data[nb_dmx_channel + 1];     // tableau de données DMX
uint8_t DMX_data_old[nb_dmx_channel + 1]; // tableau de données DMX précédentes
bool dataChanged[nb_dmx_channel + 1];     // tableau de drapeau pour donnée DMX ayant changées
unsigned long last_connected_Time = 0;    // temps de la dernière connexion DMX
unsigned long last_disconnected_Time = 0; // temps de la dernière déconnexion DMX

#define DMX_Connected_Time 1000           // temps en ms entre chaque vérification de la connexion DMX
#define DMX_Read_Time 200                 // temps en ms entre chaque mise à jour des données DMX
#define dmx_debounce_try 0                // nombre de boucle pour le debounce DMX afin d'éviter les valeurs de 0 ou 255 erronées
int dmx_count[nb_dmx_channel + 1];        // compteur de boucle pour le debounce DMX afin d'éviter les valeurs de 0 ou 255 erronées

// fonction
int readSwitch();         // fonction pour lire le codeur d'adresse DMX
void reset_dmx_counter(); // fonction pour réinitialiser le compteur de boucle pour le debounce DMX afin d'éviter les valeurs de 0 ou 255 erronées
void receiveDMX();        // fonction pour recevoir les données DMX

int readSwitch() // fonction pour lire le codeur d'adresse DMX
{
  int total = 0;
  if (digitalRead(q1) == HIGH)
  {
    total += 1;
  }
  if (digitalRead(q2) == HIGH)
  {
    total += 2;
  }
  if (digitalRead(q4) == HIGH)
  {
    total += 4;
  }
  if (digitalRead(q8) == HIGH)
  {
    total += 8;
  }
  total = ((total + 1) * 2) - 1; // correction battarde de l'adresse DMX

  return total;
}

void reset_dmx_counter() // fonction pour réinitialiser le compteur de boucle pour le debounce DMX afin d'éviter les valeurs de 0 ou 255 erronées
{
  for (int i = 0; i < nb_dmx_channel + 1; i++)
  {
    dmx_count[i] = 0;
  }
}

void receiveDMX() // fonction pour recevoir les données DMX
{
  dmx_packet_t packet;
  if (dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK))
  {

    if (!packet.err)
    {
      if (!dmxIsConnected)
      {
        Serial.println("DMX is connected!");
        last_connected_Time = millis();
        dmxIsConnected = true;
      }
      dmx_read(dmx_num, data, packet.size);
      // mise à jour des données DMX si elles ont changé et si le temps écoulé depuis la dernière mise à jour est supérieur à DMX_Read_Time
      if (millis() - lastUpdate > DMX_Read_Time)
      {
        lastUpdate = millis();          // réinitialisation du temps de la dernière mise à jour des données DMX
        last_connected_Time = millis(); // réinitialisation du temps de la dernière connexion DMX
        // mise à jour de la position
        if (DMX_data[pos_array] != data[pos_channel])
        {
          if (DMX_data[pos_array] == 0 || DMX_data[pos_array] == 255)
          {
            dmx_count[pos_array]++;
            if (dmx_count[pos_array] > dmx_debounce_try)
            {
              DMX_data[pos_array] = data[pos_channel];
              reset_dmx_counter();
              (dataChanged[pos_array]) = true;
            }
          }
          else
          {
            DMX_data[pos_array] = data[pos_channel];
            reset_dmx_counter();
            (dataChanged[pos_array]) = true;
          }
        }

        // mise à jour de la vitesse
        if (DMX_data[speed_array] != data[speed_channel])
        {
          if (DMX_data[speed_array] == 0 || DMX_data[speed_array] == 255)
          {
            dmx_count[speed_array]++;
            if (dmx_count[speed_array] > dmx_debounce_try)
            {
              DMX_data[speed_array] = data[speed_channel];
              reset_dmx_counter();
              (dataChanged[speed_array]) = true;
            }
          }
          else
          {
            DMX_data[speed_array] = data[speed_channel];
            reset_dmx_counter();
            (dataChanged[speed_array]) = true;
          }
        }
      }
    }
    else
    {
      reset_dmx_counter();
      Serial.println("A DMX error occurred.");
    }
  }
  else if (dmxIsConnected)
  {
    reset_dmx_counter();
    // Serial.println("DMX was disconnected."); // affichage de la déconnexion DMX
    if (millis() - last_connected_Time > DMX_Connected_Time)
    {
      dmxIsConnected = false;
      last_disconnected_Time = millis(); // réinitialisation du temps de la dernière déconnexion DMX
      Serial.println("DMX was halted."); // affichage de la déconnexion DMX
      //  emergencyStop = true;                    // arrêt du moteur en cas de déconnexion DMX
      //  emercency_stop_timer = millis();         // réinitialisation du timer d'arrêt d'urgence
    }
  }
}
