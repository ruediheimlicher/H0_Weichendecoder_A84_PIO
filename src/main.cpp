//
//
//
//  Created by Sysadmin on 04.04.26.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <avr/eeprom.h>

#include "defines.h"
//***********************************
/*
TO DO:
Lok-Adresse spiegeln.

RE 4/4: 0xCC erzeugt 00110011, muss 11001100 erzeugen

Trafo:         OK

Teensy-board: > spiegeln

H0-Interface:  OK

// ***********************************
// ***********************************
// FUSES
// low   E2
// high: D7
// terminal: avrdude -v  -p attiny84 -P /dev/tty.usbserial-AM0190V3 -c stk500v2 -U hfuse:w:0xDD:m -U lfuse:w:0xE2:m
// ***********************************

// ***********************************
// ***********************************


*/
//***********************************
//***********************************
uint8_t LOK_ADRESSE = 0x7F; //	11001100	Trinär
uint8_t WEICHENCODE = 0;
//***********************************
//***********************************

uint8_t SPEEDINDEX = 9;

// Ae6/6 GR 24
// uint8_t  LOK_ADRESSE = 0x33; //	00110011	Trinaer (1010)

//

/*
 commands
 LO     0x0202  // 0000 0010   0000 0010
 OPEN   0x02FE  // 0000 0010   1111 1110
 HI     0xFEFE  // 1111 1110   1111 1110
 */

#define INPORT PORTB // Input signal auf INT0
#define INPIN PINB   // Input signal

#define DATAPIN 2 // PB2, INT0

// volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
// volatile uint8_t txbuffer[buffer_size];

volatile uint8_t INT0status = 0x00;
volatile uint8_t pausestatus = 0x00;

volatile uint8_t ablaufstatus = 0x00; // Startdlay
volatile uint16_t startwaitcounter = STARTWAIT;

volatile uint8_t address = 0x00;
volatile uint8_t data = 0x00;

volatile uint8_t HIimpulsdauerPuffer = 22;  //	Puffer fuer HIimpulsdauer
volatile uint8_t HIimpulsdauerSpeicher = 0; //	Speicher  fuer HIimpulsdauer

volatile uint8_t LOimpulsdauerOK = 0;

volatile uint8_t pausecounter = 0;   //  neue �daten detektieren
volatile uint8_t abstandcounter = 0; // zweites Paket detektieren

volatile uint8_t tritposition = 0; // nummer des trit im Paket
// volatile uint8_t   lokadresse = 0;

volatile uint8_t lokadresseA = 0;
volatile uint8_t lokadresseB = 0;

volatile uint8_t deflokadresse = 0;
volatile uint8_t lokstatus = 0x00; // Funktion, Richtung

volatile uint8_t rawfunktionA = 0;
volatile uint8_t rawfunktionB = 0;

volatile uint8_t deffunktiondata = 0;

volatile uint8_t oldlokdata = 0;
// volatile uint8_t   lokdata = 0;
volatile uint8_t deflokdata = 0;

volatile uint8_t lokstatus16 = 0x00; // Funktion, Richtung

volatile uint8_t weichenstatus = 0;

volatile uint16_t weichenimpulscounter = 0;
volatile uint16_t weichewaitcounter = 0;

// volatile uint16_t   startdelaycounter = 0; //
// volatile uint16_t   newlokdata = 0;

// volatile uint16_t   blinkWait = 0x2FFF;
// volatile uint16_t   blinkOK = 0x1FFF;

volatile uint8_t rawdataA = 0;
volatile uint8_t rawdataB = 0;
// volatile uint32_t   oldrawdata = 0;

volatile uint8_t oldspeedcode = 0;
volatile uint8_t speedcode = 0;
volatile uint8_t speed = 0;

volatile uint8_t oldspeed = 0;
volatile uint8_t newspeed = 0;
volatile uint8_t minspeed = 0;   // Unterster Wert in speedlookup-tabelle
volatile uint8_t startspeed = 0; // Anlaufimpuls
volatile int8_t speedintervall = 0;

volatile uint16_t speed16 = 0;
volatile uint16_t motorPWM16 = 0;

volatile uint16_t oldspeed16 = 0;
volatile uint16_t newspeed16 = 0;
volatile uint16_t minspeed16 = 0;   // Unterster Wert in speedlookup-tabelle
volatile uint16_t startspeed16 = 0; // Anlaufimpuls
volatile uint16_t speedintervall16 = 0;

volatile uint8_t dimmcounter = 0; // LED dimmwertcounter
volatile uint8_t ledpwm = 0;      // LED PWM
volatile uint8_t ledstatus = 0;   // status LED

volatile uint8_t oldfunktion = 0;
volatile uint8_t funktion = 0;
volatile uint8_t deffunktion = 0;
volatile uint8_t waitcounter = 0;
volatile uint8_t richtungcounter = 0; // delay fuer Richtungsimpuls

volatile uint8_t pwmpin = MOTORA_PIN;      // Motor PWM
volatile uint8_t richtungpin = MOTORB_PIN; // Motor Richtung

// volatile uint8_t	Potwert=45;
//	Zaehler fuer richtige Impulsdauer
// uint8_t				Servoposition[]={23,33,42,50,60};
//  Richtung invertiert
// volatile uint8_t				Servoposition[]={60,50,42,33,23};

// volatile uint16_t	taktimpuls=0;

volatile uint8_t motorPWM = 0;

volatile uint8_t wdtcounter = 0;

volatile uint8_t taskcounter = 0;

volatile uint8_t speedlookup[15] = {}; // aktuelle speed-tabelle

uint8_t speedlookuptable[10][15] =
    {
        {0, 18, 36, 54, 72, 90, 108, 126, 144, 162, 180, 198, 216, 234, 252}, // 0
        {0, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160},   // 1
        {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140},     // 2
        {0, 7, 14, 21, 28, 35, 42, 50, 57, 64, 71, 78, 85, 92, 100},          // 3
        {0, 33, 37, 40, 44, 47, 51, 55, 58, 62, 65, 69, 72, 76, 80},          // 4

        {0, 41, 42, 44, 47, 51, 56, 61, 67, 74, 82, 90, 99, 109, 120},      // 5
        {0, 41, 43, 45, 49, 54, 60, 66, 74, 82, 92, 103, 114, 127, 140},    // 6
        {0, 41, 44, 48, 53, 59, 67, 77, 87, 99, 113, 128, 144, 161, 180},   // 7
        {0, 42, 45, 50, 57, 65, 75, 87, 101, 116, 134, 153, 173, 196, 220}, // 8
        {0, 42, 45, 51, 58, 68, 79, 93, 108, 125, 144, 165, 188, 213, 240}  // 9
};

uint8_t speedcodelookuptable[16] = {0, 0x3, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F, 0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF};

// volatile uint8_t SPEEDINDEX = 7;

volatile uint8_t maxspeed = 0; // speedlookuptable[SPEEDINDEX][14];

volatile uint8_t lastDIR = 0;
uint8_t loopledtakt = 0x40;
uint8_t refreshtakt = 0x40;
uint16_t speedchangetakt = 0x800; // takt fuer beschleunigen/bremsen

// https://stackoverflow.com/questions/70049553/best-way-to-handle-multiple-pcint-in-avr
volatile uint8_t portahistory = 0xFF; // default is high because the pull-up

void slaveinit(void)
{
   OSZIPORT |= (1 << OSZIA); // Ausgang fuer OSZI A
   OSZIDDR |= (1 << OSZIA);  // Ausgang fuer OSZI A

   LOOPLEDDDR |= (1 << LOOPLED); // HI
   LOOPLEDPORT |= (1 << LOOPLED);

   WEICHEDIP_DDR &= ~(1 << WEICHEDIP0);
   WEICHEDIP_DDR &= ~(1 << WEICHEDIP1);
   WEICHEDIP_DDR &= ~(1 << WEICHEDIP2);

   WEICHEDIP_PORT |= (1 << WEICHEDIP0); // pullup
   WEICHEDIP_PORT |= (1 << WEICHEDIP1);
   WEICHEDIP_PORT |= (1 << WEICHEDIP2);

   WEICHEDDR |= (1 << WEICHEA_PIN);   // Weichedir A
   WEICHEPORT &= ~(1 << WEICHEA_PIN); // LO

   WEICHEDDR |= (1 << WEICHEB_PIN);   // Weichedir B
   WEICHEPORT &= ~(1 << WEICHEB_PIN); // LO

   maxspeed = 254;

   pwmpin = MOTORA_PIN;
   richtungpin = MOTORB_PIN;

   // |= (1<<FIRSTRUNBIT);
}

void int0_init(void)
{

   GIMSK |= (1 << INT0);                // enable external int0
   MCUCR = (1 << ISC00 | (1 << ISC01)); // raise int0 on rising edge

   INT0status = 0;
   // sei();
   //  INT0status |= (1<<INT0_RISING);
   INT0status = 0;
   INT0status |= (1 << INT0_WAIT);
}

void timer0(uint8_t wert)
{
   // set up timer with prescaler = 1 and CTC mode
   TCCR0A = 0;
   TCCR0B = 0;

   TCCR0A |= (1 << WGM01);
   TCCR0B |= (1 << CS01);

   // initialize counter
   TCNT0 = 0;

   // initialize compare value
   OCR0A = wert; //

   // clear interrupt flag as a precaution
   TIFR0 |= 0x01;

   // enable compare interrupt
   TIMSK0 |= (1 << OCIE0A);

   // enable global interrupts
   // sei();
}

// MARK: ISR(EXT_INT0_vect)
ISR(EXT_INT0_vect)
{
   // OSZI_A_LO();
   // if(displayfenstercounter%4 == 0)
   {
      // OSZI_B_LO();

      if (INT0status == 0) // neue Daten beginnen
      {

         // OSZI_A_HI();
         INT0status |= (1 << INT0_START);
         INT0status |= (1 << INT0_WAIT); // delay, um Wert des Eingangs zum richtigen Zeitpunkt zu messen

         INT0status |= (1 << INT0_PAKET_A); // erstes Paket lesen
         // OSZIPORT &= ~(1<<PAKETA);

         pausecounter = 0;   // pausen detektieren, reset fuer jedes HI
         abstandcounter = 0; // zweites Paket detektieren,

         waitcounter = 0;
         tritposition = 0;
         funktion = 0;
      }

      else // Data im Gang, neuer Interrupt
      {
         INT0status |= (1 << INT0_WAIT);

         pausecounter = 0;
         abstandcounter = 0;
         waitcounter = 0;
         //     OSZIALO;
      }
   }
   /*
   else
   {
      INT0status = 0;
   }
   */
}

// MARK: ISR Timer0
ISR(TIM0_COMPA_vect) // Schaltet Impuls an MOTORB_PIN LO wenn speed
{
   if (weichenstatus & (1 << WEICHESTART)) //Impuls noch ON
   {
      weichenimpulscounter++;
      if (weichenimpulscounter > WEICHENIMPULSDAUER)
      {
         weichenstatus &= ~(1 << ABLENKUNG);
         weichenstatus &= ~(1 << GERADE);
         WEICHEPORT &= ~(1 << WEICHEA_PIN);
         WEICHEPORT &= ~(1 << WEICHEB_PIN);

         //
         weichenstatus &= ~(1 << WEICHESTART);

         weichenstatus |= (1 << WEICHEWAIT);
         

      }
   }
   

   // MARK: TIMER0 TIMER0_COMPA INT0
   if (INT0status & (1 << INT0_WAIT))
   {
      waitcounter++;
      if (waitcounter > 2) // Impulsdauer > minimum, nach einer gewissen Zeit den Stautus abfragen
      {

         // OSZI_A_LO();
         // OSZIAHI;
         INT0status &= ~(1 << INT0_WAIT);
         if (INT0status & (1 << INT0_PAKET_A))
         {
            // OSZI_B_LO();
            if (tritposition < 8) // Adresse)
            {
               if (INPIN & (1 << DATAPIN)) // Pin HI,
               {
                  lokadresseA |= (1 << tritposition); // bit ist 1
               }
               else //
               {
                  lokadresseA &= ~(1 << tritposition); // bit ist 0
               }
            }
            else if (tritposition < 10) // Funktion
            {
               if (INPIN & (1 << DATAPIN)) // Pin HI,
               {
                  rawfunktionA |= (1 << (tritposition - 8)); // bit ist 1
               }
               else //
               {
                  rawfunktionA &= ~(1 << (tritposition - 8)); // bit ist 0
               }
            }

            else
            {
               if (INPIN & (1 << DATAPIN)) // Pin HI,
               {
                  rawdataA |= (1 << ((tritposition - 10))); // bit ist 1
               }
               else //
               {
                  rawdataA &= ~(1 << (tritposition - 10)); // bit ist 0
               }
            }
         }

         if (INT0status & (1 << INT0_PAKET_B))
         {
            if (tritposition < 8) // Adresse)
            {

               if (INPIN & (1 << DATAPIN)) // Pin HI,
               {
                  lokadresseB |= (1 << tritposition); // bit ist 1
               }
               else //
               {
                  lokadresseB &= ~(1 << tritposition); // bit ist 0
               }
            }
            else if (tritposition < 10) // bit 8,9: funktion
            {
               if (INPIN & (1 << DATAPIN)) // Pin HI,
               {
                  rawfunktionB |= (1 << (tritposition - 8)); // bit ist 1
               }
               else //
               {
                  rawfunktionB &= ~(1 << (tritposition - 8)); // bit ist 0
               }
            }

            else
            {
               if (INPIN & (1 << DATAPIN)) // Pin HI,
               {
                  rawdataB |= (1 << (tritposition - 10)); // bit ist 1
               }
               else
               {
                  rawdataB &= ~(1 << (tritposition - 10)); // bit ist 0
               }
            }

            if (!(lokadresseB == LOK_ADRESSE))
            {
            }
         }

         if (tritposition < 17)
         {
            tritposition++;
         }
         else // Paket gelesen
         {
            // Paket A?
            if (INT0status & (1 << INT0_PAKET_A)) // erstes Paket, Werte speichern
            {

               oldfunktion = funktion;

               INT0status &= ~(1 << INT0_PAKET_A); // Bit fuer erstes Paket weg
               INT0status |= (1 << INT0_PAKET_B);  // Bit fuer zweites Paket setzen
               tritposition = 0;
            }
            else if (INT0status & (1 << INT0_PAKET_B)) // zweites Paket, Werte testen
            {
               // SYNC_LO();
               // displaystatus |= (1<<DISPLAY_GO);
               //  // Displayfenster begin

               // displayfenstercounter = MAXFENSTERCOUNT;
               //  MARK: EQUAL
               if (lokadresseA && ((rawfunktionA == rawfunktionB) && (rawdataA == rawdataB) && (lokadresseA == lokadresseB))) // Lokadresse > 0 und Lokadresse und Data OK
               {

                  // SYNC_LO();
                  if (lokadresseB == LOK_ADRESSE)
                  {

                     // weichenstatus |= (1<<WEICHERUN);

                     // OSZI_A_LO();
                     //  TEST1_LO();
                     // OSZI_B_LO();
                     //  Daten uebernehmen

                     lokstatus |= (1 << ADDRESSBIT);
                     deflokadresse = lokadresseB;
                     // deffunktion = (rawdataB & 0x03); // bit 0,1 funktion als eigene var
                     deffunktion = rawfunktionB;

                     if (deffunktion)
                     {
                        lokstatus |= (1 << FUNKTIONBIT);
                        ledstatus |= (1 << LED_CHANGEBIT); // change setzen
                     }
                     else
                     {
                        lokstatus &= ~(1 << FUNKTIONBIT);
                        ledstatus |= (1 << LED_CHANGEBIT); // led-change setzen
                     }
                     // deflokdata aufbauen
                     for (uint8_t i = 0; i < 8; i++)
                     {
                        // if ((rawdataB & (1<<(2+i))))
                        if ((rawdataB & (1 << i)))
                        {
                           deflokdata |= (1 << i);
                        }
                        else
                        {
                           deflokdata &= ~(1 << i);
                        }
                     }

                     // Weichennummer checken
                     WEICHENCODE = 0xFF;
                     WEICHENCODE = WEICHEDIP_PIN & 0x07;

                     WEICHENCODE = 7 - WEICHENCODE; // dipschalter ist active LOW > invertieren

                     if (deflokdata == speedcodelookuptable[WEICHENCODE]) // Weiche passt
                     {
                        weichenimpulscounter = 0;
                        if (!(weichenstatus & (1 << WEICHESTART)))
                        {
                           weichenstatus |= (1 << WEICHESTART);
                           weichenimpulscounter = 0;
                           // OSZI_B_LO();

                           if (lokstatus & (1 << FUNKTIONBIT)) // Weiche auf Ablenkung stellen
                           {
                              weichenstatus |= (1 << ABLENKUNG);
                              weichenstatus &= ~(1 << GERADE);
                              WEICHEPORT &= ~(1 << WEICHEA_PIN);
                              WEICHEPORT |= (1 << WEICHEB_PIN);
                           }
                           else // Weiche auf Gerade stellen
                           {
                              weichenstatus |= (1 << GERADE);
                              weichenstatus &= ~(1 << ABLENKUNG);
                              WEICHEPORT |= (1 << WEICHEA_PIN);
                              WEICHEPORT &= ~(1 << WEICHEB_PIN);
                           }
                        }

                     }
                     else
                     {
                        /*
                        weichenstatus &= ~(1<<ABLENKUNG);
                        weichenstatus &= ~(1<<GERADE);
                        WEICHEPORT &= ~(1<<WEICHEA_PIN);
                        WEICHEPORT &= ~(1<<WEICHEB_PIN);
                        //weichenstatus |= (1<<WEICHEOFF);
                        */
                     }
                  }
                  else
                  {
                     // aussteigen
                     INT0status = 0;

                     return;
                  }

               } // if (lokadresseA &&...
               else
               {
                  lokstatus &= ~(1 << ADDRESSBIT);

                  INT0status = 0;
                  return;
               }

               INT0status |= (1 << INT0_END);
               if (INT0status & (1 << INT0_PAKET_B))
               {
                  //               TESTPORT |= (1<<TEST2);
               }
            } // End Paket B
         }
         // OSZI_B_HI();
      } // waitcounter > 2
   } // if INT0_WAIT

   if (INPIN & (1 << DATAPIN)) // Pin HI, input   im Gang
   {
      //      HIimpulsdauer++; // zaehlen
   }
   else // LO, input fertig, Bilanz
   {
      if (abstandcounter < 20)
      {
         abstandcounter++;
      }
      else // if (abstandcounter ) // Paket 2
      {
         abstandcounter = 0;
         // OSZIAHI;
      }

      if (pausecounter < 120)
      {
         pausecounter++; // pausencounter incrementieren
      }
      else
      {
         pausecounter = 0;
         INT0status = 0; // Neue Daten abwarten
         return;
      }

   } // input LO
   // OSZI_B_HI();
} // TIM0

int main(void)
{
   // WDT ausschalten
   MCUSR = 0;
   wdt_disable();
   //   lastDIR = 1;
   slaveinit();

   int0_init();
   _delay_ms(2);

   timer0(4);
   uint16_t loopcount0 = 0;
   uint16_t loopcount1 = 0;

   _delay_ms(2);
   oldfunktion = 0x03; // 0x02
   oldlokdata = 0xCC;  //

   // WDT
   // https://bigdanzblog.wordpress.com/2015/07/20/resetting-rebooting-attiny85-with-watchdog-timer-wdt/

   wdt_reset();
   ledpwm = LEDPWM;

   uint8_t i = 0;
   for (i = 0; i < 15; i++)
   {
      // speedlookup[i] = speedlookuptable[SPEEDINDEX][i]; // soeedlookup fuer Lok laden
   }

   sei();
   while (1)
   {
      // Timing: loop: 40 us, takt 85us, mit if-teil 160 us
      wdt_reset();
      {

         loopcount1++;
         if (loopcount1 >= speedchangetakt)
         {
            LOOPLEDPORT ^= (1 << LOOPLED); // Kontrolle lastDIR
            loopcount1 = 0;
            // OSZIATOG;

         } // loopcount1 >= speedchangetakt

      } // Source OK

      /*
      if (weichenstatus & (1 << WEICHESTART))
      {

         //weichenimpulscounter++;

         if (weichenstatus & (1 << ABLENKUNG))
         {
            //WEICHEPORT &= ~(1 << WEICHEA_PIN);
            //WEICHEPORT |= (1 << WEICHEB_PIN);
            //weichenstatus &= ~(1 << ABLENKUNG);
         }
         else if (weichenstatus & (1 << GERADE))
         {
            //WEICHEPORT |= (1 << WEICHEA_PIN);
            //WEICHEPORT &= ~(1 << WEICHEB_PIN);
            //weichenstatus &= ~(1 << GERADE);
         }

         if (weichenimpulscounter > WEICHENIMPULSDAUER)
         {
            weichenstatus &= ~(1 << ABLENKUNG);
            weichenstatus &= ~(1 << GERADE);
            WEICHEPORT &= ~(1 << WEICHEA_PIN);
            WEICHEPORT &= ~(1 << WEICHEB_PIN);

            //
            weichenstatus &= ~(1 << WEICHESTART);
         }
      }
      */
      loopcount0++;
      if (loopcount0 >= refreshtakt)
      {
         // OSZIATOG;
         // LOOPLEDPORT ^= (1<<LOOPLED);

         loopcount0 = 0;

      } // loopcount0>=refreshtakt

      // OSZIAHI;
   } // while
   return 0;
}
